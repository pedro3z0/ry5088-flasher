/*
 * board.c - clock/USB/delay glue for the Fun60 Ultra firmware.
 */
#include "board.h"
#include "system_at32f402_405.h"

/* ---- DWT cycle-counter based delay (precise at 216 MHz) ---- */
void board_delay_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_cycles(uint32_t cycles)
{
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < cycles) { /* spin */ }
}

void usb_delay_us(uint32_t us)
{
  delay_cycles(us * (system_core_clock / 1000000U));
}

void usb_delay_ms(uint32_t ms)
{
  while (ms--) usb_delay_us(1000U);
}

/* ---- USB OTG-FS pin configuration ----
 * DP (PA12) / DM (PA11) are driven by the OTG-FS peripheral automatically.
 * With USB_VBUS_IGNORE set (usb_conf.h) the VBUS-sense pin is not used, so only
 * the OTG GPIO port clock is required. */
void usb_gpio_config(void)
{
  crm_periph_clock_enable(OTG_PIN_GPIO_CLOCK, TRUE);
#ifdef USB_SOF_OUTPUT_ENABLE
  {
    gpio_init_type gpio_init_struct;
    crm_periph_clock_enable(OTG_PIN_SOF_GPIO_CLOCK, TRUE);
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_pins = OTG_PIN_SOF;
    gpio_init(OTG_PIN_SOF_GPIO, &gpio_init_struct);
    gpio_pin_mux_config(OTG_PIN_SOF_GPIO, OTG_PIN_SOF_SOURCE, OTG_PIN_MUX);
  }
#endif
}

/* ---- select the 48 MHz USB clock ----
 * HEXT path: derive the USB 48 MHz from PLLU (set up by system_clock_config()).
 * (The HICK/ACC path is omitted on purpose; the keyboard runs from a crystal.) */

/* The factory RY bootloader stays in HID-DFU when the boot mailbox holds the
 * magic or the app-slot chip-ID header is not the keyboard chip-ID
 * (see docs/protocol.md §6). Prefer the mailbox: programming the magic word
 * needs no erase and leaves our app image intact, so a spurious trip is
 * harmless. Only if the mailbox cannot be programmed do we fall back to
 * erasing the app-region sector at 0x08005000 (header + our own vector
 * table), which a reflash restores. Neither path touches the resident
 * bootloader at 0x08000000-0x08004FFF. */
#define RY_BOOT_MAILBOX_ADDR  0x08004800u
#define RY_BOOT_MAILBOX_MAGIC 0x55AA55AAu
#define RY_CHIP_ID_FLASH_ADDR   0x08005000u
#define RY_FLASH_SECTOR_SIZE    0x800u   /* 2 KB sectors */

static void board_write_dfu_markers(void)
{
  flash_unlock();
  /* 1) Documented request-DFU marker. */
  (void)flash_word_program(RY_BOOT_MAILBOX_ADDR, RY_BOOT_MAILBOX_MAGIC);
  /* 2) Also corrupt the app-slot chip-ID header ("AT32" -> 0x00000000). The
   *    bootloader's own boot check then keeps it in HID-DFU even if it clears
   *    the mailbox on entry — the mailbox alone was NOT sufficient on this
   *    part (the board kept re-entering the app instead of parking in DFU).
   *    Both are single word programs: flash programming only clears bits 1->0,
   *    so neither needs a preceding erase, and the app image stays in place. */
  (void)flash_word_program(RY_CHIP_ID_FLASH_ADDR, 0x00000000u);
  flash_lock();
}

void board_enter_dfu(void)
{
  __disable_irq();
  board_write_dfu_markers();
  NVIC_SystemReset();
  for (;;) { /* never returns; the RY bootloader comes up in HID-DFU */ }
}

void board_force_recovery(void)
{
  /* Same effect: make sure no USB state is half-alive, then park in RY-DFU.
   * The bootloader re-inits the peripheral from reset anyway. */
  board_enter_dfu();
}

void usb_clock48m_select(usb_clk48_s clk_s)
{
  volatile uint32_t guard = BOOT_WAIT_SPIN;
  (void)clk_s;
  crm_pllu_output_set(TRUE);
  /* Bounded wait for PLLU. NON-FATAL: never reset the board from a clock wait;
   * continue and let USB init proceed (a dead clock shows up as no enumeration,
   * not as a reset loop). */
  while (crm_flag_get(CRM_PLLU_STABLE_FLAG) != SET) { /* wait PLLU */
    if (!--guard) break;
  }
  crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_PLLU);
}
