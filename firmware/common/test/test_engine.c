/*
 * test_engine.c - host-side self-test for the pure-logic engine (hall + hid_report).
 * Builds natively (no AT32/USB). Run via `make test`.
 */
#include <stdio.h>
#include <string.h>
#include "hall.h"
#include "hid_report.h"
#include "keyscan.h"
#include "board_keymap.h"

static int fails = 0;
#define CHECK(cond, msg) do { \
  if (!(cond)) { printf("  FAIL: %s\n", msg); fails++; } \
  else         { printf("  ok  : %s\n", msg); } } while (0)

/* feed one frame where every key reads `def` except key `idx` which reads `val`. */
static void frame(hall_engine_t *e, uint8_t *pressed, unsigned idx, uint16_t val, uint16_t def)
{
  uint16_t raw[KS_NUM_KEYS];
  for (unsigned i = 0; i < KS_NUM_KEYS; i++) raw[i] = def;
  raw[idx] = val;
  hall_process(e, raw, pressed);
}

int main(void)
{
  hall_engine_t e;
  uint8_t pressed[KS_NUM_KEYS];
  const unsigned K = 9;            /* key index 9 = 'A' (0x04) in the default map */
  const uint16_t REST = 3000, DEEP = 1000;

  printf("[1] Normal-mode actuation + hysteresis\n");
  hall_init(&e);
  frame(&e, pressed, K, REST, REST);                 /* prime baseline */
  CHECK(pressed[K] == 0, "released after baseline seed");
  frame(&e, pressed, K, DEEP, REST);                 /* full press */
  CHECK(pressed[K] == 1, "press at 2000 counts actuates");
  /* travel 700 counts is in the (release 600, actuation 800) hysteresis band */
  frame(&e, pressed, K, 2300, REST);
  CHECK(pressed[K] == 1, "stays pressed in hysteresis band (700 counts)");
  frame(&e, pressed, K, 2750, REST);                 /* travel 250 < release 600 */
  CHECK(pressed[K] == 0, "releases once travel drops below release point");

  printf("[2] Rapid Trigger: release/re-press on direction reversal\n");
  hall_init(&e);
  hall_keycfg_t rt = { .mode = HALL_MODE_RAPID_TRIGGER, .press_cmm = 200,
                       .release_cmm = 150, .rt_press_cmm = 50, .rt_release_cmm = 50 };
  hall_set_global(&e, &rt);
  frame(&e, pressed, K, REST, REST);                 /* prime */
  frame(&e, pressed, K, DEEP, REST);                 /* press deep */
  CHECK(pressed[K] == 1, "RT press on downstroke");
  frame(&e, pressed, K, 1500, REST);                 /* up 500 counts (>0.5mm) */
  CHECK(pressed[K] == 0, "RT release on upstroke past rt_release");
  frame(&e, pressed, K, 1200, REST);                 /* down 300 counts (>0.5mm) */
  CHECK(pressed[K] == 1, "RT re-press on downstroke past rt_press");

  printf("[2b] Rapid Trigger honours the configured actuation point on first press\n");
  hall_init(&e);
  hall_set_global(&e, &rt);                          /* act 800, rt_press 200 counts */
  frame(&e, pressed, K, REST, REST);                 /* prime (travel 0) */
  frame(&e, pressed, K, 2600, REST);                 /* travel 400 < actuation 800 */
  CHECK(pressed[K] == 0, "RT does NOT actuate before the actuation point (travel 400)");
  frame(&e, pressed, K, 2100, REST);                 /* travel 900 >= actuation 800 */
  CHECK(pressed[K] == 1, "RT actuates once travel reaches the actuation point (900)");
  frame(&e, pressed, K, REST, REST);                 /* full release (travel 0 <= release) */
  CHECK(pressed[K] == 0, "RT releases and re-arms on full release");
  frame(&e, pressed, K, 2600, REST);                 /* travel 400 < actuation 800 again */
  CHECK(pressed[K] == 0, "re-armed: the next first press again needs the actuation point");

  printf("[2c] Traditional vs Continuous RT diverge above the actuation point\n");
  {
    /* act = 1200 counts, rel = 600, rt_press = rt_release = 200 counts. */
    hall_keycfg_t c = { .mode = HALL_MODE_RAPID_TRIGGER, .press_cmm = 300,
                        .release_cmm = 150, .rt_press_cmm = 50, .rt_release_cmm = 50 };

    /* ---- traditional: rising above the actuation point disarms -> next
     * trigger must re-cross actuation (which 950 does NOT reach). ---- */
    c.rt_mode = HALL_RT_TRADITIONAL;
    hall_init(&e);
    hall_set_global(&e, &c);
    frame(&e, pressed, K, REST, REST);     /* d = 0    */
    frame(&e, pressed, K, 1000, REST);     /* d = 2000 -> press, arm      */
    CHECK(pressed[K] == 1, "traditional: press at 2000");
    frame(&e, pressed, K, 2300, REST);     /* d = 700, above act -> release + disarm */
    CHECK(pressed[K] == 0, "traditional: released on the upstroke");
    frame(&e, pressed, K, 2050, REST);     /* d = 950 (<act 1200), +250 vs ref 700   */
    CHECK(pressed[K] == 0, "traditional: 950 does NOT re-trigger (must reach 1200)");

    /* ---- continuous: stays armed across the whole travel -> the same 950
     * downward move re-triggers. ---- */
    c.rt_mode = HALL_RT_CONTINUOUS;
    hall_init(&e);
    hall_set_global(&e, &c);
    frame(&e, pressed, K, REST, REST);
    frame(&e, pressed, K, 1000, REST);
    frame(&e, pressed, K, 2300, REST);
    CHECK(pressed[K] == 0, "continuous: released on the upstroke");
    frame(&e, pressed, K, 2050, REST);
    CHECK(pressed[K] == 1, "continuous: 950 re-triggers above the actuation point");
  }

  printf("[3] Boot report: modifiers fold, keycodes fill, de-dupe\n");
  {
    uint8_t pr[KS_NUM_KEYS]; memset(pr, 0, sizeof pr);
    uint8_t rep[HID_BOOT_REPORT_LEN];
    pr[1] = 1;   /* Esc    0x29 */
    pr[4] = 1;   /* LShift 0xE1 -> modifier bit1 */
    pr[9] = 1;   /* A      0x04 */
    uint8_t n = hid_build_boot_report(pr, keymap_default, rep);
    CHECK(n == 2, "two non-modifier keys");
    CHECK(rep[0] == 0x02, "LShift folded into modifier byte (bit1)");
    CHECK(rep[1] == 0x00, "reserved byte zero");
    int has_esc = 0, has_a = 0;
    for (int i = 2; i < 8; i++) { if (rep[i] == 0x29) has_esc = 1; if (rep[i] == 0x04) has_a = 1; }
    CHECK(has_esc && has_a, "Esc + A present in keycode slots");
  }

  printf("[4] Boot report: >6 keys -> ErrorRollOver\n");
  {
    uint8_t pr[KS_NUM_KEYS]; memset(pr, 0, sizeof pr);
    uint8_t rep[HID_BOOT_REPORT_LEN];
    /* 7 distinct letter keys: indices 7('1'),13('2'),19('3'),25('4'),31('5'),37('6'),43('7') */
    unsigned ks[7] = {7,13,19,25,31,37,43};
    for (int i = 0; i < 7; i++) pr[ks[i]] = 1;
    hid_build_boot_report(pr, keymap_default, rep);
    int rollover = 1;
    for (int i = 2; i < 8; i++) if (rep[i] != 0x01) rollover = 0;
    CHECK(rollover, "all six slots = 0x01 ErrorRollOver");
  }

  printf("\n%s (%d failure%s)\n", fails ? "TESTS FAILED" : "ALL TESTS PASSED",
         fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
