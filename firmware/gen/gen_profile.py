#!/usr/bin/env python3
"""Generate the board-specific C for one keyboard from its profile.

    python3 gen/gen_profile.py profiles/<dev_id>.toml <out_dir>

Reads a profile TOML (merged over the platform defaults below) and writes three
files into <out_dir>, which the shared firmware in common/ compiles against:

    board_config.h    the board-specific values the firmware consumes
    board_keymap.h    the default keymap table
    board_led_map.c   the LED serpentine map + palette

board_config.h carries only what the code actually parameterizes: identity,
matrix geometry, the Hall/switch model, LED counts, and the wireless flag. The
GPIO pin/port wiring is the vendor reference design and stays compiled-in
(common/keyscan.c, rgb.c, wireless.c); a profile may document it under [matrix],
[rgb] and [wireless] for the record, but those pin fields are not emitted here.

Output is deterministic; stdlib only (Python 3.11+ for tomllib).
"""
import sys
import os

try:
    import tomllib
except ModuleNotFoundError:  # Python < 3.11
    try:
        import tomli as tomllib  # type: ignore
    except ModuleNotFoundError:
        sys.exit("error: need Python 3.11+ (tomllib) or `pip install tomli`")

# ---- platform / switch defaults (a minimal profile may omit these) ----------
HALL_DEFAULTS = {
    "press_decreases": True, "full_travel_cmm": 400, "assumed_span": 1600,
    "noise_counts": 16, "press_cmm": 200, "release_cmm": 150,
    "rt_press_cmm": 50, "rt_release_cmm": 50,
    # Rapid Trigger flavour: "traditional" (classic; trigger zone only below the
    # actuation point) or "continuous" (stock CRT; whole travel until full
    # release). The stock FUN60 family ships "continuous"; boards may override.
    "rt_mode": "continuous",
}
# magnetism factory defaults presented over the vendor protocol (switch-level),
# in hundredths of a mm.
MAG_DEFAULTS = {"press": 200, "lift": 280, "rt_press": 50, "rt_lift": 50}
# shared 8-colour palette (chain colours); overridable via [leds].palette.
PALETTE = [
    [0xFF, 0x00, 0x00], [0x00, 0xFF, 0x00], [0x00, 0x00, 0xFF], [0xFF, 0x80, 0x00],
    [0xFF, 0x00, 0xFF], [0xFF, 0xFF, 0x00], [0x96, 0x96, 0x96], [0x00, 0x00, 0x00],
]


def die(msg):
    sys.exit(f"error: {msg}")


def need(profile, section, key):
    if section not in profile or key not in profile[section]:
        die(f"profile is missing required [{section}].{key}")
    return profile[section][key]


def cstr(s):
    return '"' + str(s).replace("\\", "\\\\").replace('"', '\\"') + '"'


def hexb(v):
    return f"0x{int(v) & 0xFF:02X}"


def hex4(v):
    return f"0x{int(v) & 0xFFFF:04X}"


def fmt_table(values, per_line, fmt):
    rows = []
    for i in range(0, len(values), per_line):
        rows.append("  " + " ".join(fmt(v) + "," for v in values[i:i + per_line]))
    return "\n".join(rows)


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: gen_profile.py <profile.toml> <out_dir>")
    profile_path, out_dir = sys.argv[1], sys.argv[2]
    with open(profile_path, "rb") as f:
        p = tomllib.load(f)

    meta = p.get("meta", {})
    usb = p.get("usb", {})
    ver = p.get("version", {})
    leds = p.get("leds", {})
    wl = p.get("wireless", {})
    hall = dict(HALL_DEFAULTS)
    hall.update(p.get("hall", {}))
    mag = dict(MAG_DEFAULTS)
    keymap = p.get("keymap", {}).get("codes", [])

    dev_id = need(p, "meta", "dev_id")
    if not (isinstance(dev_id, int) and 0 <= dev_id <= 0xFFFF):
        die(f"[meta].dev_id must be an integer 0..65535, got {dev_id!r}")
    stem = os.path.splitext(os.path.basename(profile_path))[0]
    if stem.isdigit() and int(stem) != dev_id:
        die(f"filename profiles/{stem}.toml does not match [meta].dev_id = {dev_id} "
            f"(profiles are named by dev_id)")
    cols = need(p, "matrix", "cols")
    rows = need(p, "matrix", "rows")
    sites = cols * rows
    if keymap and len(keymap) != sites:
        die(f"[keymap].codes has {len(keymap)} entries, expected cols*rows = {sites}")

    led_count = need(p, "leds", "count")
    phys_col = need(p, "leds", "phys_col")
    phys_row = need(p, "leds", "phys_row")
    for name, arr in (("phys_col", phys_col), ("phys_row", phys_row)):
        if len(arr) != led_count:
            die(f"[leds].{name} has {len(arr)} entries, expected count = {led_count}")
    palette = leds.get("palette", PALETTE)

    product = usb.get("product_str", meta.get("display_name", ""))
    wl_identity = wl.get("identity") or product
    if len(wl_identity) > 31:
        die(f"[wireless].identity is {len(wl_identity)} chars; must be <= 31 (it is sent in a "
            f"33-byte frame as '<identity>-<slot>')")
    cfg = f"""/* Generated from profiles/{os.path.basename(profile_path)} by gen/gen_profile.py — DO NOT EDIT. */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* ---- identity --------------------------------------------------------- */
#define BOARD_DEV_ID            {dev_id}u                /* GET_INFOR id (decimal) */
#define MONSGEEK_VENDOR_ID      {hex4(usb.get("vid", 0x3151))}
#define MONSGEEK_PRODUCT_ID     {hex4(usb.get("pid", 0x5030))}
#define MONSGEEK_BCD_DEVICE     {hex4(usb.get("bcd_device", 0x0603))}
#define MONSGEEK_INFO_DEVID_LO  {hexb(dev_id)}
#define MONSGEEK_INFO_DEVID_HI  {hexb(dev_id >> 8)}
#define MONSGEEK_INFO_VER_HI    {hexb(ver.get("hi", 0))}
#define MONSGEEK_INFO_VER_LO    {hexb(ver.get("lo", 0))}
#define MONSGEEK_STR_MANUFACTURER {cstr(usb.get("manufacturer_str", "MonsGeek Keyboard"))}
#define MONSGEEK_STR_PRODUCT      {cstr(product)}
#define MONSGEEK_STR_CONFIG       {cstr(usb.get("config_str", "Keyboard Config"))}
#define MONSGEEK_STR_INTERFACE    {cstr(usb.get("interface_str", "Keyboard Interface"))}
#define WL_IDENTITY_STR           {cstr(wl_identity)}

/* ---- matrix geometry -------------------------------------------------- */
#define KS_COLS                 {cols}u
#define KS_ROWS                 {rows}u
#define MG_NUM_SITES            {sites}u

/* ---- hall / switch ---------------------------------------------------- */
#define HALL_PRESS_DECREASES    {1 if hall["press_decreases"] else 0}
#define HALL_FULL_TRAVEL_CMM    {hall["full_travel_cmm"]}u
#define HALL_ASSUMED_SPAN       {hall["assumed_span"]}u
#define HALL_NOISE_COUNTS       {hall["noise_counts"]}
#define HALL_DEF_PRESS_CMM      {hall["press_cmm"]}
#define HALL_DEF_RELEASE_CMM    {hall["release_cmm"]}
#define HALL_DEF_RT_PRESS_CMM   {hall["rt_press_cmm"]}
#define HALL_DEF_RT_RELEASE_CMM {hall["rt_release_cmm"]}
#define HALL_DEF_RT_MODE        {1 if hall["rt_mode"] == "continuous" else 0}u  /* 0 = traditional, 1 = continuous */
#define MG_DEF_MAG_PRESS        {mag["press"]}
#define MG_DEF_MAG_LIFT         {mag["lift"]}
#define MG_DEF_MAG_RT_PRESS     {mag["rt_press"]}
#define MG_DEF_MAG_RT_LIFT      {mag["rt_lift"]}

/* ---- LEDs ------------------------------------------------------------- */
#define MG_NUM_LEDS             {led_count}u
#define MG_LED_MAX_COL          {leds.get("max_col", cols)}u
#define MG_LED_MAX_ROW          {leds.get("max_row", rows - 1)}u

/* ---- wireless --------------------------------------------------------- */
#define BOARD_WIRELESS          {1 if meta.get("wireless", False) else 0}

#endif /* BOARD_CONFIG_H */
"""

    km = f"""/* Generated from profiles/{os.path.basename(profile_path)} — DO NOT EDIT. */
#ifndef BOARD_KEYMAP_H
#define BOARD_KEYMAP_H
#include <stdint.h>
#include "keyscan.h"

/* HID usage IDs, indexed by key = col*KS_ROWS + row. 0xE0..0xE7 are modifiers. */
static const uint8_t keymap_default[KS_NUM_KEYS] = {{
{fmt_table(keymap, rows, hexb) if keymap else "  0"}
}};

#endif /* BOARD_KEYMAP_H */
"""

    pal = ",\n".join("  { " + ", ".join(hexb(c) for c in row) + " }" for row in palette)
    lm = f"""/* Generated from profiles/{os.path.basename(profile_path)} — DO NOT EDIT. */
#include "led_layout.h"

/* chain index -> physical column */
const uint8_t led_phys_col[MG_NUM_LEDS] = {{
{fmt_table(phys_col, 26, lambda v: f"{v:2d}")}
}};

/* chain index -> physical row */
const uint8_t led_phys_row[MG_NUM_LEDS] = {{
{fmt_table(phys_row, 25, lambda v: f"{v:2d}")}
}};

const uint8_t led_palette[8][3] = {{
{pal}
}};
"""

    os.makedirs(out_dir, exist_ok=True)
    for fname, text in (("board_config.h", cfg), ("board_keymap.h", km), ("board_led_map.c", lm)):
        with open(os.path.join(out_dir, fname), "w") as f:
            f.write(text)
    print(f"generated board_config.h, board_keymap.h, board_led_map.c "
          f"for dev_id {dev_id} ({meta.get('display_name', '?')}) -> {out_dir}")


if __name__ == "__main__":
    main()
