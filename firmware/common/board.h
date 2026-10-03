/*
 * board.h - minimal board support for the Fun60 Ultra firmware
 * (clock-48M select for USB, OTG GPIO, and a DWT-based microsecond delay).
 * Replaces the AT-START board package (no LEDs/buttons on the real keyboard).
 */
#ifndef MONSGEEK_BOARD_H
#define MONSGEEK_BOARD_H

#include "at32f402_405.h"
#include "usb_conf.h"

/* Spin budget for each bounded boot wait (HEXT ready, PLL stable, PLLU stable,
 * USB CONFIGURED). Order of tens of millions of iterations — seconds at 216
 * MHz, so a real crystal has ample time and a dead clock path gives up
 * promptly instead of hanging forever with no USB. */
#define BOOT_WAIT_SPIN  30000000u

void board_delay_init(void);
void usb_gpio_config(void);
void usb_clock48m_select(usb_clk48_s clk_s);

/* Host requested DFU (the `0x7F 55 AA 55 AA` enter-bootloader handshake): write
 * the boot mailbox + invalidate the app-slot chip-ID, then reset, so the
 * resident bootloader stays in HID-DFU. Mirrors what the stock app does — a
 * bare reset (no mailbox) makes the bootloader simply re-enter the app, which
 * is why a flash would "only restart" and never reach `3151:502A`.
 * Never returns. */
void board_enter_dfu(void);

/* Last-resort self-recovery (never returns). Writes the factory boot mailbox
 * (`0x55AA55AA` at `0x08004800`); the resident RY bootloader then stays in
 * HID-DFU (`3151:502A`), from which `ry-flash --image <stock_fw_at32.bin>
 * --arm` restores with no BOOT0 strap. The resident RY bootloader at
 * `0x08000000`–`0x08004FFF` is never touched. */
void board_force_recovery(void);

#endif /* MONSGEEK_BOARD_H */
