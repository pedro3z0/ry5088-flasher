/*
 * hall.c - per-key Hall processing: baseline calibration, travel, actuation and
 *          Rapid Trigger. Pure logic (no hardware).
 */
#include "hall.h"

/* HALL_NOISE_COUNTS and the per-key defaults (HALL_DEF_*) come from board_config.h. */

static void hall_seed(hall_key_t *k, uint16_t raw)
{
  k->baseline = raw;
  k->extreme  = raw;
  k->travel   = 0;
  k->rt_ref   = 0;
  k->pressed  = 0;
  k->primed   = 1;
  k->rt_armed = 0;
}

void hall_init(hall_engine_t *e)
{
  const hall_keycfg_t d = {
    .mode = HALL_MODE_NORMAL,
    .rt_mode = HALL_DEF_RT_MODE,
    .press_cmm = HALL_DEF_PRESS_CMM,
    .release_cmm = HALL_DEF_RELEASE_CMM,
    .rt_press_cmm = HALL_DEF_RT_PRESS_CMM,
    .rt_release_cmm = HALL_DEF_RT_RELEASE_CMM,
  };
  for (unsigned i = 0; i < KS_NUM_KEYS; i++) {
    e->cfg[i] = d;
    e->key[i].primed = 0;
  }
}

void hall_set_global(hall_engine_t *e, const hall_keycfg_t *cfg)
{
  for (unsigned i = 0; i < KS_NUM_KEYS; i++)
    e->cfg[i] = *cfg;
}

/* pressed-positive deflection from the released baseline (>=0 means pressed). */
static int hall_deflection(const hall_key_t *k, uint16_t raw)
{
#if HALL_PRESS_DECREASES
  return (int)k->baseline - (int)raw;
#else
  return (int)raw - (int)k->baseline;
#endif
}

void hall_process(hall_engine_t *e, const uint16_t *raw, uint8_t *pressed)
{
  for (unsigned i = 0; i < KS_NUM_KEYS; i++) {
    hall_key_t *k = &e->key[i];
    const hall_keycfg_t *c = &e->cfg[i];
    uint16_t r = raw[i];

    if (!k->primed) {
      hall_seed(k, r);
      if (pressed) pressed[i] = 0;
      continue;
    }

    /* --- released-rest baseline + deepest-press extreme tracking --- */
#if HALL_PRESS_DECREASES
    if (r > k->baseline) k->baseline = r;
    if (r < k->extreme)  k->extreme  = r;
#else
    if (r < k->baseline) k->baseline = r;
    if (r > k->extreme)  k->extreme  = r;
#endif

    int d = hall_deflection(k, r);
    if (d < 0) d = 0;

    /* follow slow downward drift of the rest level when sitting near rest */
    if (!k->pressed && d <= HALL_NOISE_COUNTS) {
#if HALL_PRESS_DECREASES
      if (k->baseline > r) k->baseline -= 1;
#else
      if (k->baseline < r) k->baseline += 1;
#endif
    }
    k->travel = (int16_t)d;

    int act = (int)HALL_CMM_TO_COUNTS(c->press_cmm);
    int rel = (int)HALL_CMM_TO_COUNTS(c->release_cmm);
    if (rel >= act) rel = act - (int)HALL_CMM_TO_COUNTS(20);  /* force hysteresis  */
    if (rel < 0) rel = 0;

    if (c->mode == HALL_MODE_RAPID_TRIGGER) {
      int rtp = (int)HALL_CMM_TO_COUNTS(c->rt_press_cmm);
      int rtl = (int)HALL_CMM_TO_COUNTS(c->rt_release_cmm);

      /* ---- HALL_RT_TRADITIONAL (classic / Wooting-style Rapid Trigger) ----
       * The active trigger zone is ONLY the band below the actuation point.
       *  - A press actuates once travel reaches `act` (and then "arms" RT).
       *  - While the key rises but stays below `act`, a downward reversal of
       *    at least `rtp` re-triggers it; an upward reversal of at least `rtl`
       *    releases it.
       *  - Rising back ABOVE the actuation point (`d < act`) resets the key and
       *    DISARMS RT, so the next trigger must cross the actuation point again.
       * `release_cmm` is a hard release gate at the bottom of the band. */
      if (c->rt_mode == HALL_RT_TRADITIONAL) {
        if (d < act) k->rt_armed = 0;          /* above actuation -> reset+disarm */
        if (!k->pressed) {
          if (d <= k->rt_ref) k->rt_ref = (int16_t)d;   /* follow the valley */
          int trip = k->rt_armed ? (d - k->rt_ref >= rtp) : (d >= act);
          if (trip) { k->pressed = 1; k->rt_armed = 1; k->rt_ref = (int16_t)d; }
        } else {
          if (d >= k->rt_ref) k->rt_ref = (int16_t)d;   /* follow the peak */
          if (k->rt_ref - d >= rtl) { k->pressed = 0; k->rt_ref = (int16_t)d; }
        }
        if (d <= rel) {                        /* dropped to the release floor */
          k->pressed = 0;
          k->rt_armed = 0;
          k->rt_ref = (int16_t)d;
        }
      } else {
        /* ---- HALL_RT_CONTINUOUS (stock CRT) ----
         * After the first actuation the whole travel stays "ready" until full
         * release: the key re-triggers anywhere (even above the actuation point)
         * on a downward move of `rt`. This is the stock FUN60 Pro behaviour, so
         * the vendor app's RT control keeps working. */
        if (!k->pressed) {
          if (d <= k->rt_ref) k->rt_ref = (int16_t)d;   /* follow the valley */
          int trip = k->rt_armed ? (d - k->rt_ref >= rtp) : (d >= act);
          if (trip) { k->pressed = 1; k->rt_armed = 1; k->rt_ref = (int16_t)d; }
        } else {
          if (d >= k->rt_ref) k->rt_ref = (int16_t)d;   /* follow the peak */
          if (k->rt_ref - d >= rtl) { k->pressed = 0; k->rt_ref = (int16_t)d; }
        }
        if (d <= rel) {                        /* released past the release point */
          k->pressed = 0;
          k->rt_armed = 0;
          k->rt_ref = (int16_t)d;
        }
      }
    } else {
      /* Normal + (DKS/ModTap/Toggle/SnapTap fall back to) fixed threshold. */
      if (!k->pressed) { if (d >= act) k->pressed = 1; }
      else             { if (d <= rel) k->pressed = 0; }
      k->rt_ref = (int16_t)d;
    }

    if (pressed) pressed[i] = k->pressed;
  }
}
