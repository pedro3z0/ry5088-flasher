/*
 * main.c - MonsGeek Fun60 Ultra firmware (wired + optional wireless).
 *
 * Builds on the USB device stack + the analog key-scan / Hall engine so the
 * board boots, types, lights up and is fully configurable by the MonsGeek app:
 *
 *   keyscan_frame()  -> raw[84] ADC samples
 *   hall_process()   -> pressed[84]   (per-key actuation from the magnetism cfg)
 *   hid_build_boot_report() + usbd_ept_send(0x81)
 *   led_effects_render() + rgb_show()  -> WS2812 RGB (SPI2 + DMA1, see rgb.c)
 *   persist_load/save()                -> config survives reboot (EFC, see persist.c)
 *
 * The vendor feature-report path (vendor_proto.c) round-trips every exposed
 * setting; SET handlers mark the config dirty and the main loop flushes it to
 * internal flash. Clean factory defaults: ANSI keymap, 2.0 mm actuation, Normal
 * mode, Wave lighting at full brightness.
 *
 * Links into the application slot @0x08005200 behind the factory bootloader.
 */
#include "at32f402_405.h"
#include "at32f402_405_clock.h"
#include "board.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbd_int.h"
#include "usbd_core.h"
#include "monsgeek_class.h"
#include "monsgeek_desc.h"
#include "descriptors.h"

#include "keyscan.h"
#include "hall.h"
#include "hid_report.h"
#include "board_keymap.h"

#include "vendor_proto.h"
#include "led_effects.h"
#include "rgb.h"
#include "persist.h"
#include "wireless.h"

otg_core_type otg_core_struct;

/* Link mode latched at boot from PC13: 0 = wired/USB, 1 = wireless 2.4G/BLE. */
static int g_wireless = 0;

static hall_engine_t  hall;
static uint16_t       raw[KS_NUM_KEYS];
static uint8_t        pressed[KS_NUM_KEYS];
static uint8_t        report[HID_BOOT_REPORT_LEN];
static uint8_t        last_report[HID_BOOT_REPORT_LEN];
static led_frame_t    frame;

#define LED_TICK_MS        16u     /* ~60 Hz RGB refresh                        */
#define SAVE_DEBOUNCE_MS   400u    /* batch rapid SETs, save when settled       */

/* If USB never reaches CONFIGURED after boot (e.g. a descriptor/clock mistake
 * in a new build), bail to the RY HID-DFU bootloader instead of sitting USB-
 * dead: no BOOT0 strap needed for recovery. Rounded-up spin budget (~seconds
 * at 216 MHz on a live crystal, generous for slow hosts). */
#define USB_BRINGUP_GUARD  0x0C000000u

/* monotonic millisecond clock from the DWT cycle counter (handles 32-bit wrap
 * as long as it is sampled far more often than ~19.9 s, which the loop does). */
static uint32_t now_ms(void)
{
  static uint32_t last_cyc = 0, acc_ms = 0, rem = 0;
  uint32_t per_ms = system_core_clock / 1000u;
  uint32_t cyc = DWT->CYCCNT;
  uint32_t d = cyc - last_cyc;
  last_cyc = cyc;
  rem += d;
  acc_ms += rem / per_ms;
  rem %= per_ms;
  return acc_ms;
}

/* Map the per-key magnetism config (app-programmable) onto the Hall engine.
 * raw units are centi-mm (raw 200 = 2.00 mm). The magnetism arrays cover 64
 * keys; physical scan sites beyond that reuse the last entry (best-effort - the
 * exact site<->magnetism-key index map is a hardware bring-up item). */
static void apply_magnetism(const monsgeek_state_t *st, hall_engine_t *e)
{
  unsigned i;
  for (i = 0; i < KS_NUM_KEYS; i++) {
    unsigned k = (i < MG_NUM_MAG_KEYS) ? i : (MG_NUM_MAG_KEYS - 1u);
    uint16_t press = st->mag_press[k];
    uint16_t lift  = st->mag_lift[k];
    /* Hall release point must sit below the actuation point for hysteresis; the
     * app "lift travel" is used when it does, otherwise a sane margin is kept. */
    uint16_t rel = (lift < press) ? lift : (press >= 50u ? (uint16_t)(press - 50u)
                                                          : (uint16_t)(press / 2u));
    /* Vendor mode byte: low 7 bits = base mode, bit 0x80 = the orthogonal RT
     * ("fire") flag. The engine implements Normal and Rapid Trigger; the other
     * base modes fall back to Normal until they get engine support. */
    uint8_t mb = st->mag_mode[k];
    e->cfg[i].mode           = (mb & 0x80u) ? HALL_MODE_RAPID_TRIGGER : HALL_MODE_NORMAL;
    /* RT flavour is global (EXTENSION byte cfg[0x1F]); default from the profile. */
    e->cfg[i].rt_mode        = (st->cfg[0x1f] & 1u) ? HALL_RT_CONTINUOUS
                                                    : HALL_RT_TRADITIONAL;
    e->cfg[i].press_cmm      = press;
    e->cfg[i].release_cmm    = rel;
    e->cfg[i].rt_press_cmm   = st->mag_rt_press[k];
    e->cfg[i].rt_release_cmm = st->mag_rt_lift[k];
  }
}

/* Render the configured effect into the WS2812 chain (Off if LED disabled). */
static void render_leds(const monsgeek_state_t *st, uint32_t t_ms)
{
  led_params_t lp;
  int led_on = monsgeek_get_led_params(st, &lp);
  if (!led_on) lp.mode = LED_MODE_OFF;
  led_effects_render(&lp, st->userpic, t_ms, frame);
  rgb_show(frame);
}

static void send_boot_report(uint8_t *rep)
{
  if (usbd_connect_state_get(&otg_core_struct.dev) != USB_CONN_STATE_CONFIGURED)
    return;
  usbd_ept_send(&otg_core_struct.dev, MG_EP_BOOT_KBD_IN, rep, HID_BOOT_REPORT_LEN);
}

static int report_changed(const uint8_t *a, const uint8_t *b)
{
  for (unsigned i = 0; i < HID_BOOT_REPORT_LEN; i++) if (a[i] != b[i]) return 1;
  return 0;
}

int main(void)
{
  monsgeek_state_t *st;
  uint32_t led_t = 0, dirty_t = 0;
  int prev_configured = 0;

  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  system_clock_config();          /* 216 MHz; PLLU -> 48 MHz USB clock          */
  board_delay_init();             /* DWT cycle counter (also the ms time base)  */

  /* ---- link-mode select: read PC13 = VBUS/cable sense. PC13 HIGH selects the
   * wired/USB path, PC13 LOW selects wireless. The wired path below is unchanged;
   * a LOW PC13 additionally brings up SPI3. Wired-only boards (profile
   * wireless = false) skip the detect and stay wired.                             */
#if BOARD_WIRELESS
  {
    gpio_init_type io;
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    gpio_default_para_init(&io);
    io.gpio_pins = GPIO_PINS_13;            /* GPIOC bit 0x2000 = PC13               */
    io.gpio_mode = GPIO_MODE_INPUT;
    /* Pull-DOWN so an unplugged board reads LOW => wireless; VBUS drives it HIGH
     * when a cable is present. The pull direction is a bring-up item. */
    io.gpio_pull = GPIO_PULL_DOWN;
    gpio_init(GPIOC, &io);
    g_wireless = (gpio_input_data_bit_read(GPIOC, GPIO_PINS_13) == RESET) ? 1 : 0;
  }
#else
  g_wireless = 0;                           /* wired-only board                     */
#endif

  /* USB device bring-up */
  usb_gpio_config();
  crm_periph_clock_enable(OTG_CLOCK, TRUE);
  usb_clock48m_select(USB_CLK_HEXT);
  nvic_irq_enable(OTG_IRQ, 0, 0);
  usbd_init(&otg_core_struct,
            USB_SPEED_CORE_ID,        /* follows -DUSB_OTG_HS/_FS (stock = HS) */
            USB_ID,
            &monsgeek_class_handler,
            &monsgeek_desc_handler);

  /* Safety net: give the host time to configure USB. BOUNDED but NON-FATAL —
   * a diagnostic build must never reset the board itself, so if USB does not
   * reach CONFIGURED we simply continue and let the vendor protocol / engine
   * run anyway. (Only the host's explicit 0x7F handshake resets us.) */
  {
    volatile uint32_t guard = USB_BRINGUP_GUARD;
    while (usbd_connect_state_get(&otg_core_struct.dev) != USB_CONN_STATE_CONFIGURED) {
      if (!--guard) break;
    }
  }

  /* engine + RGB bring-up */
  keyscan_init();
  hall_init(&hall);
  rgb_init();

  /* wireless 2.4G/BLE bus bring-up (only when PC13 selected wireless). The
   * keyscan/hall/RGB/persist paths below are identical in both modes; only the
   * report-send site differs. The pairing / connection state machine is not
   * ported here - bring-up item. */
  if (g_wireless) wireless_init();

  /* live vendor settings live inside the USB class struct */
  st = &((monsgeek_class_t *)otg_core_struct.dev.class_handler->pdata)->state;

  /* boot config: clean defaults if flash is blank, else the saved image. This
   * runs before enumeration so the LEDs light even with no host attached. */
  persist_load(&flash_efc_ops, st);
  apply_magnetism(st, &hall);

  for (unsigned i = 0; i < HID_BOOT_REPORT_LEN; i++) last_report[i] = 0;

  /* (See the USB-CONFIGURED gate right after usbd_init: it runs before
   * keyscan/RGB/persist bring-up, so a USB-dead build bails before those can
   * hang in turn.) */

  while (1)
  {
    uint32_t now = now_ms();
    int configured =
        (usbd_connect_state_get(&otg_core_struct.dev) == USB_CONN_STATE_CONFIGURED);

    /* host requested the 0x7F 55 AA 55 AA enter-bootloader handshake.
     * Must write the boot mailbox (and invalidate the app chip-ID) before
     * resetting, otherwise the resident bootloader just re-enters this app and
     * the host never sees 502A — i.e. a flash that "only restarts". */
    if (st->enter_bootloader) { usb_delay_ms(20); board_enter_dfu(); }

    /* On each (re)enumeration the shared class_init resets the state to defaults;
     * reload the persisted config so flash stays the source of truth. */
    if (configured && !prev_configured) {
      persist_load(&flash_efc_ops, st);
      apply_magnetism(st, &hall);
    }
    prev_configured = configured;

    /* ---- one scan / actuation / report cycle ---- */
    keyscan_frame(raw);
    hall_process(&hall, raw, pressed);
    /* Uses the default keymap: live keymatrix remap is persisted and round-tripped
     * but not yet applied here (see docs/firmware.md feature status). */
    hid_build_boot_report(pressed, keymap_default, report);
    if (report_changed(report, last_report)) {
      if (g_wireless) wl_send_keyboard(report);   /* wireless SPI3 path (0x81) */
      else            send_boot_report(report);   /* wired USB path            */
      for (unsigned i = 0; i < HID_BOOT_REPORT_LEN; i++) last_report[i] = report[i];
    }

    /* ---- RGB refresh ---- */
    if ((uint32_t)(now - led_t) >= LED_TICK_MS) {
      led_t = now;
      render_leds(st, now);
    }

    /* ---- apply config changes immediately; persist to flash debounced ---- */
    if (st->cfg_dirty) {
      apply_magnetism(st, &hall);           /* take effect now, not only after the save */
      if (dirty_t == 0) dirty_t = now ? now : 1u;
      if ((uint32_t)(now - dirty_t) >= SAVE_DEBOUNCE_MS) {
        persist_save(&flash_efc_ops, st);   /* clears cfg_dirty */
        dirty_t = 0;
      }
    } else {
      dirty_t = 0;
    }
  }
}

/* OTG-FS global interrupt -> USB device IRQ handler */
void OTG_IRQ_HANDLER(void)
{
  usbd_irq_handler(&otg_core_struct);
}
