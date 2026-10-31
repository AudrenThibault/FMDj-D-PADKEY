/* SPDX-License-Identifier: GPL-3.0-only
 * LSDj fork: 2026 DEADACTIVE */
/* The LSDj unit (FELUCCA_LSDJ=1): its own object, linked in place of the BLE test unit. The core's unit is
 * compiled exactly as in the BLE test build (FELUCCA_BLE=1: its main-loop hooks ble_key / ble_service / ble_draw,
 * the console's `ble`), so the update path is the one proven on the device (gate 2b,
 * tools/check_update_path.py). The core calls the hooks below; this unit calls the core only through ble_core.h.
 *
 * Nothing runs at boot: PLAY starts the Game Boy (ble_key). From then on, each main-loop pass runs one Game Boy
 * frame when the audio ring has room for its sound (the audio ISR plays the ring: lsdj_audio), then sends a few
 * changed line pairs to the screen (1.5x: 240 x 216). The status bar (ENV menu) tells how it keeps up: FPS, CPU, GAPS,
 * the seconds in which the ring ran dry. The emulator is reached only through function pointers set
 * by lsdj_start (gate 4, tools/check_ble_boot.py --lsdj, walks the direct calls from boot).
 *
 * Songs move over the console: `ble WORDS` is a transfer command (lsdj_xfer.c; web/lsdj.html drives it). The host
 * holds the Game Boy between two frames, reads or writes the save RAM, and lets it run again.
 *
 * Flash (lsdj_store.c: Felucca's user-sample area only): PLAY loads LSDj's save from it (reading only); it is
 * written when REC is held for a second, and for a request the console queued (`ble save`, the kit banks). Both
 * happen in lsdj_step, which the core reaches only through its pointer: no flash write is reachable from boot
 * (gate 4). Custom kits live there too, and show in the ROM's free kit banks (rom_tab).
 *
 * This firmware holds nothing of LSDj. The user's own ROM and its start-up save are the cartridge block
 * (lsdj_cart.c) at the end of the app's flash slot, put there by the install page or the Mac build and read on
 * PLAY; without one PLAY says so and nothing runs. A firmware with a block in it holds that user's ROM: LSDj's
 * licence forbids distributing it, so it is never published (build/ is git-ignored). */
#include "ble/ble_core.h"
#ifndef LSDJ_POOL
#define LSDJ_POOL __attribute__((section(".pool")))   /* big zero-initialised buffers (app.ld POOL) */
#endif
#include "lsdj/lsdj_gb.c"
#include "lsdj/lsdj_font.h"
#include "lsdj/lsdj_emu.c"
#include "lsdj/lsdj_xfer.c"
#define LSDJ_STORE_FN static __attribute__((noinline))   /* gate 4 finds the flash writers by name */
#include "lsdj/lsdj_store.c"
#include "lsdj/lsdj_cart.c"
#define LSDJ_ROM_BANKS LSDJ_CART_BANKS        /* the most a ROM has; the block says how many this one has */

#define RING_N 4096u                          /* stereo frames: 93 ms, 5.5 Game Boy frames */
#define RING_LOW (RING_N / 2u)                /* under this the step runs a second Game Boy frame in its pass */
#define Y0_BAR 4u                             /* the picture: rows 4 .. 219, the status bar below it */
#define Y0_FULL 12u                           /* ... without the status bar: in the middle, rows 12 .. 227, and the
                                               * 12 rows above and below in the theme's lightest shade */
#define BORDER 12u                            /* (AF_S, the status font, is 12 px: a note fits in the lower one) */
#define GROUPS (LSDJ_H / 2u)                  /* line pairs: 3 screen rows each */
#define PUSH_MAX 8u                           /* line pairs merged into one transfer (24 rows, <= 11.5 KB) */
#define STATUS_Y 223u
#define MENU_Y 96u                            /* the menu (ENV) covers screen rows 96 .. MENU_END - 1; the picture
                                               * above it runs on */
#define MENU_LINE 16u
#define MENU_END (MENU_Y + 1u + MENU_LINE * (LSDJ_NMI + 1u))

static uint8_t sram[LSDJ_SRAM] LSDJ_POOL;
static uint8_t fb[LSDJ_H][LSDJ_W] LSDJ_POOL, shown[LSDJ_H][LSDJ_W] LSDJ_POOL;
static uint8_t dirty[LSDJ_H];                 /* rows changed since they were sent (set by emu_line) */
static int16_t ring_buf[2u * RING_N] LSDJ_POOL;
static uint16_t pushbuf[2][PUSH_MAX * 3u * 240u] LSDJ_POOL;   /* DMA sources: one filled while the other goes out */
static lsdj_ring_t ring;
static void (*volatile run)(void);            /* lsdj_step once started */
static void (*volatile draw)(void);           /* lsdj_push once started */
static uint32_t group, buf_i, splash_done, splash_ms;
#define SPLASH_MS 3000u                       /* the start screen, then LSDj starts by itself (PLAY: at once) */
static uint32_t pad_map[8];                   /* button bit per Game Boy key (JOYPAD order), from KEYS */
static lsdj_xfer_t xfer;                      /* the console's `ble ...` (lsdj_xfer.c): the page moves songs with it */
static char xfer_out[LSDJ_XFER_OUT];

/* The Game Boy keys (the owner's layout, 2026-10-12):
 *   the cursor: the SELECT knob left / right, the ALGORITHM knob up / down (clockwise: right, down), a step = a press
 *   A, B: the top F and G of the keyboard (lsdj_gb.c LSDJ_NOTE_A / _B);  START: PLAY;  SELECT: SEQ
 *   LSDj's screens (SELECT + direction): the PRESETS knob left / right, OCT+ up, OCT- down
 * DMGo (2026-10-17): A, B on D#5 and F#5, the 16 white keys its steps, no screen shortcuts (SELECT + a direction
 * means other things in it); the rest the same.
 * and, as before, the cursor on buttons too (they can be held): UP SCL, DOWN SAVE, LEFT HOME, RIGHT ARP.
 * The buttons below by label, in JOYPAD bit order: A, B (no button), SELECT, START, RIGHT, LEFT, UP, DOWN.
 * OCT- + OCT+ held 5 s together is still the core's UBOOT. */
static const int8_t KEYS[8] = {-1, -1, CORE_BTN_SEQ, CORE_BTN_PLAY,
                               CORE_BTN_ARP, CORE_BTN_HOME, CORE_BTN_SCL, CORE_BTN_SAVE};
static lsdj_knobs_t knobs;                    /* knob steps and shortcuts waiting to be pressed (lsdj_gb.c) */
static const lsdj_flash_t flash = {core_flash_ptr, core_flash_erase, core_flash_prog};
static const lsdj_flash_t flash_ro = {core_flash_ptr, 0, 0};   /* PLAY's: it reads, and has nothing to write with */
static lsdj_store_t store;                    /* what the flash holds (lsdj_store.c), scanned by lsdj_start */
static uint8_t flash_on;                      /* the core has flash for us */
static lsdj_cart_t cart;                      /* the cartridge block: the ROM's banks and the start-up save */
static lsdj_bank_t rom_tab[LSDJ_ROM_BANKS];   /* the Game Boy's ROM: the block's, the flash kit banks in its free ones */
static int prog = LSDJ_PROG_NONE;             /* the program installed (lsdj_gb.c lsdj_prog): LSDj or DMGo */
static uint32_t save_n;                       /* its save RAM: 128 KiB (LSDj) or 32 KiB (DMGo), the start of sram[] */
static uint8_t with_steps;                    /* DMGo v1: the white keys are its steps (lsdj_steps_*) */
static lsdj_steps_t steps;
static uint8_t kit_bank[LSDJ_ST_KMAX], nkit;  /* the ROM's free kit banks in order: flash kit bank j shows as kit_bank[j] */
static uint8_t kit_buf[LSDJ_BANK] LSDJ_POOL;  /* a kit bank on its way from the console to flash */

/* RGB565, byte-swapped as the panel takes them: shade 0 (light) .. 3 (dark) */
#define SW(c) ((uint16_t)(((c) >> 8) | ((c) << 8)))
#define RGB(r, g, b) SW((((r) >> 3) << 11) | (((g) >> 2) << 5) | ((b) >> 3))
static uint16_t pal[4];                       /* the screen theme's (the menu; lsdj_gb.c LSDJ_THEME_RGB) */

/* GLO: KEYBOARD or D-PAD, as OPLBend Tracker's. In KEYBOARD the note keys are a keyboard, no Game Boy key (in
 * DMGo the white keys are its steps); in D-PAD they are the pad (lsdj_gb.c LSDJ_DPAD_NOTE). Not kept: KEYBOARD at
 * power-on. */
static uint8_t dpad;
static uint32_t notes_mask;                   /* note keys held when the mode changed: not pressed until let go */
static lsdj_rep_t rep;                        /* the D-PAD cross's repeat (lsdj_gb.c) */

static int glo_led(void)                       /* as OPLBend Tracker's: lit in D-PAD; in KEYBOARD it blinks, the glow
                                                * then dark, 0.4 s each (where to go for D-PAD) */
{
    return dpad ? CORE_LED_LIT : (core_ms() / 400u) & 1u ? CORE_LED_GLOW : CORE_LED_OFF;
}

static void dpad_leds(void)                    /* as OPLBend Tracker's: GLO (glo_led); the keyboard's keys glow, all
                                                * of them in KEYBOARD, the pad's in D-PAD */
{
    uint32_t pad_keys = 0, i;
    for (i = 0; i < 8u; i++)
        pad_keys |= LSDJ_DPAD_NOTE[i];
    core_led_button(CORE_BTN_GLO, glo_led());
    for (i = 0; i < 27u; i++)
        core_led_note((int)i, !dpad || ((pad_keys >> i) & 1u) ? CORE_LED_GLOW : CORE_LED_OFF);
}

/* The menu, on ENV (lsdj_gb.c lsdj_menu_*): open, it covers the picture's lower half and takes the keys; the Game Boy
 * plays on. Its settings are the layout record's in flash (lsdj_store.c), read at PLAY, written in the step when
 * the menu closes with something changed (never from the key handler: gate 4). */
static lsdj_menu_t menu;
static lsdj_set_t set_saved;                  /* what flash holds */
static uint8_t menu_paint_req, menu_last, set_req, bar_shown;
static uint8_t menu_mask;                     /* keys down when the menu closed: not the Game Boy's until let go */
static uint32_t y0 = Y0_BAR;                  /* the picture's first screen row (0 zoomed, Y0_BAR, Y0_FULL) */
static uint8_t zoomed;                        /* the picture as drawn (geometry_set): ZOOM (lsdj_gb.c LSDJ_ZW) */
static uint32_t g_lines = 2u, g_n = GROUPS;   /* its groups: Game Boy lines, how many (line pairs; zoomed: tile rows) */
static uint32_t ztop = LSDJ_ZHEAD;            /* zoomed: the first tile row shown under the header (lsdj_zoom_top) */
static uint32_t zfollow_frame;                /* ... the Game Boy frame the cursor was last looked for in */
static uint8_t border_req;                    /* the borders to paint (1 the upper, 2 the lower): no status bar */

static struct {
    uint32_t frames, waits, doubles, frame_max_us, frame_us_sum, pushes, start_ms, push_ticks, push_calls;
    uint32_t sec_ms, sec_frames, sec_under, last_fps, last_under, sec_us, last_load;
    uint32_t win_ms, win_frames, last_speed;  /* SPD: over 4 s windows (whole frames in 1 s jitter by 1.7 %) */
    uint32_t last_fps10;                      /* FPS on the status bar, tenths: over the same windows */
    uint32_t buttons, notes;                  /* as read for the last frame (console `ble`) */
    int32_t pot, knob;                        /* the MASTER pot: last reading, its filter state */
    volatile uint32_t master_q12;             /* its gain, applied by the audio ISR */
    uint8_t pad, pad_shown;                   /* the Game Boy keys of the last frame; those on the status line */
    uint8_t held_shown;                       /* "USB" is on the status line */
    char note[24];                            /* shown there instead for a while: SAVING, SAVED nnK, SAVE FULL, SAVE
                                               * FAILED (2 s); LSDJ SAVE KEPT, DMGO SAVE KEPT (8 s) */
    uint32_t note_ms, note_for, rec_ms;       /* since the note, for how long; since REC went down (| 1: 0 = up) */
    uint8_t rec_done;                         /* this REC press has saved */
    uint32_t sec_skipped, last_skipped;       /* LSDj's wait loop: instructions not run, a frame (console `ble`) */
    uint32_t step_ticks, sec_frame_max, sec_pass_max, last_frame_max, last_pass_max;   /* the last second's longest
                                               * frame and longest time between two steps, us (console `ble`) */
    uint32_t gaps;                            /* seconds in which the sound ran dry (the status line's GAPS) */
    uint8_t gap_off;                          /* ... but not this one: the Game Boy was held, started or saved in it */
    int error;
} st;

/* ---- the audio ISR's source (core_audio.c snd_block, FELUCCA_LSDJ): Q15 stereo ---- */
void lsdj_audio(int32_t *out, uint32_t n)
{
    uint32_t i;
    if (!run) {
        for (i = 0; i < 2u * n; i++)
            out[i] = 0;
        return;
    }
    lsdj_ring_pull(&ring, out, n);
    lsdj_gain(out, n, st.master_q12);         /* the MASTER pot */
}

/* ---- text ---- */
static char *put_s(char *b, const char *s)
{
    while (*s)
        *b++ = *s++;
    *b = 0;
    return b;
}
static char *put_u(char *b, uint32_t v)
{
    char t[12];
    uint32_t n = 0;
    do
        t[n++] = (char)('0' + v % 10u);
    while (v /= 10u);
    while (n)
        *b++ = t[--n];
    *b = 0;
    return b;
}

static char *put_hex(char *b, uint32_t v)
{
    uint32_t i = 28u;
    while (i && !(v >> i))
        i -= 4u;
    for (;; i -= 4u) {
        *b++ = "0123456789ABCDEF"[(v >> i) & 15u];
        if (!i)
            break;
    }
    *b = 0;
    return b;
}

/* the status bar (the menu's STATUS BAR, off at first). Left: FPS = Game Boy frames per second (59.7 is real
 * time); CPU = the emulator's time per frame against a Game Boy frame (16.74 ms): over 100 % it cannot keep up,
 * under 100 % it waits for the audio */
static void status_line(void)
{
    char b[48], *p = b;
    if (!menu.set.bar || zoomed)
        return;
    p = put_s(p, "FPS ");
    p = put_u(p, st.last_fps10 / 10u);
    p = put_s(p, ".");
    p = put_u(p, st.last_fps10 % 10u);
    p = put_s(p, " CPU ");
    p = put_u(p, st.last_load);
    put_s(p, "%");
    core_text(0, (int)STATUS_Y, 112, CORE_FONT_S, b, CORE_MID, CORE_BG, 0);
    bar_shown = 1;
}

/* right: USB while the host holds the Game Boy; else the Game Boy keys held, or with none GAPS: the seconds so far
 * in which the sound ran dry (the FM-1 was too slow: a gap is heard), and each option that is switched off:
 * SLOW (the ROM read, LFO), FULL (every frame drawn, FX), SPIN (LSDj's wait loop run, EDIT), OLD (the sound per
 * frame, ENV) */
static void keys_line(void)
{
    char b[40], *p;
    st.pad_shown = st.pad;
    st.held_shown = xfer.held;
    if (!menu.set.bar || zoomed)              /* (a note shows in the middle: ann_show) */
        return;
    bar_shown = 1;
    if (xfer.held) {
        put_s(b, "USB");
    } else if (st.pad == 0xFFu) {
        p = put_u(put_s(b, "GAPS "), st.gaps);
        if (!(emu_mode & EMU_FAST_ROM))
            p = put_s(p, " SLOW");
        if (!(emu_mode & EMU_FRAME_SKIP))
            p = put_s(p, " FULL");
        if (!(emu_mode & EMU_WAIT_SKIP))
            p = put_s(p, " SPIN");
        if (!(emu_mode & EMU_SND_FINE))
            put_s(p, " OLD");
    } else {
        lsdj_pad_text(b, st.pad);
    }
    core_text(112, (int)STATUS_Y, 128, CORE_FONT_S, b, CORE_THEME, CORE_BG, 0);
}

/* ---- the picture's geometry: zoomed (tile rows: the header 10 screen rows each, then 10 of 22 from ztop; 2x wide)
 * or not (line pairs, 3 rows each, 1.5x) ---- */
static int geometry_set(void)                  /* from the settings; 1: it changed (then everything is to be sent) */
{
    uint8_t z = menu.set.zoom != 0;
    uint32_t y = z ? 0u : menu.set.bar ? Y0_BAR : Y0_FULL;
    border_req = z || menu.set.bar ? 0 : 3u;
    if (y == y0 && z == zoomed)
        return 0;
    y0 = y;
    zoomed = z;
    g_lines = z ? 8u : 2u;
    g_n = LSDJ_H / g_lines;
    ztop = LSDJ_ZHEAD;
    group = 0;
    return 1;
}
static uint32_t gx(uint32_t x) { return zoomed ? LSDJ_ZX(x) : LSDJ_SX(x); }   /* Game Boy x -> screen x */
static uint32_t g_h(uint32_t g)                /* a group's screen rows (0: zoomed, not in the view) */
{
    if (!zoomed)
        return 3u;
    if (g < LSDJ_ZHEAD)
        return LSDJ_ZHEAD_H;
    return g >= ztop && g < ztop + LSDJ_ZROWS ? LSDJ_ZROW_H : 0u;
}
static uint32_t g_y(uint32_t g)                /* ... its first screen row (when shown) */
{
    if (!zoomed)
        return y0 + 3u * g;
    return g < LSDJ_ZHEAD ? g * LSDJ_ZHEAD_H : LSDJ_ZHEAD * LSDJ_ZHEAD_H + (g - ztop) * LSDJ_ZROW_H;
}
static int group_dirty(uint32_t g)             /* (one out of the view waits: sent when it comes into it) */
{
    uint32_t r, d = 0;
    if (!g_h(g))
        return 0;
    for (r = g * g_lines; r < (g + 1u) * g_lines; r++)
        d |= dirty[r];
    return d != 0;
}
static int group_span(uint32_t g, uint32_t *x0, uint32_t *x1)   /* changed: 1 and the Game Boy columns [x0, x1) */
{
    return zoomed ? lsdj_span_n(fb[8u * g], shown[8u * g], 8u, LSDJ_ZW, x0, x1) : lsdj_span(fb[2u * g], shown[2u * g], x0, x1);
}
static uint32_t menu_group(void)               /* the first group the open menu covers */
{
    uint32_t g;
    if (!zoomed)
        return (MENU_Y - y0) / 3u;
    for (g = 0; g < g_n; g++)
        if (g_h(g) && g_y(g) + g_h(g) > MENU_Y)
            return g;
    return g_n;
}
static void rows_resend(uint32_t ylo, uint32_t yhi)   /* the Game Boy lines on screen rows ylo .. yhi: sent again */
{
    uint32_t g;
    for (g = 0; g < g_n; g++)
        if (g_h(g) && g_y(g) <= yhi && g_y(g) + g_h(g) > ylo) {
            memset(shown[g * g_lines], 0xFF, g_lines * LSDJ_W);
            memset(dirty + g * g_lines, 1, g_lines);
        }
}
static void zoom_follow(void)                  /* zoomed: the view after LSDj's cursor, once a Game Boy frame */
{
    uint32_t lo, hi, top;
    if (!zoomed || st.frames == zfollow_frame)
        return;
    zfollow_frame = st.frames;
    if (!lsdj_cursor_rows(fb, &lo, &hi))
        return;
    top = lsdj_zoom_top(ztop, lo, hi);
    if (top == ztop)
        return;
    ztop = top;                               /* moved: all under the header sent again */
    memset(shown[LSDJ_ZHEAD * 8u], 0xFF, (LSDJ_H - LSDJ_ZHEAD * 8u) * LSDJ_W);
    memset(dirty + LSDJ_ZHEAD * 8u, 1, LSDJ_H - LSDJ_ZHEAD * 8u);
}

/* ---- the menu (ENV) ---- */
static void theme_apply(void)                  /* the theme's colours; the whole picture is sent again */
{
    uint32_t i;
    const uint8_t (*c)[3] = LSDJ_THEME_RGB[menu.set.theme];
    for (i = 0; i < 4u; i++)
        pal[i] = RGB(c[i][0], c[i][1], c[i][2]);
    memset(shown, 0xFF, sizeof shown);
    memset(dirty, 1, sizeof dirty);
}

static void layout_apply(void)                 /* zoomed, or the picture up with the status bar, in the middle without */
{
    if (geometry_set()) {                     /* moved: the screen cleared, everything sent again */
        core_fill(0, 0, 240, 240, CORE_BG);
        memset(shown, 0xFF, sizeof shown);
        memset(dirty, 1, sizeof dirty);
        bar_shown = 0;
        menu_paint_req = 1;
    }
}

static void settings_apply(void)               /* the menu's settings into the emulator, the screen, the status bar */
{
    emu_mode = (EMU_FAST_ROM | EMU_FRAME_SKIP | EMU_WAIT_SKIP | EMU_SND_FINE) & ~(uint32_t)menu.set.off;
    theme_apply();
    layout_apply();
    if (menu.set.bar) {
        status_line();
        keys_line();
    } else {
        keys_line();                          /* (blanks the strip) */
    }
}

static void menu_paint(void)                   /* the box over the picture's lower half: a title, a line a setting */
{
    char label[16], value[16];
    uint32_t i, y;
    core_fill(0, (int)MENU_Y, 240, (int)(MENU_END - MENU_Y), CORE_BG);
    core_text(0, (int)MENU_Y + 1, 240, CORE_FONT_S, "MENU  (ENV CLOSES)", CORE_THEME, CORE_BG, 1);
    for (i = 0; i < LSDJ_NMI; i++) {
        int sel = i == menu.item;
        y = MENU_Y + 1u + MENU_LINE * (i + 1u);
        lsdj_menu_line(&menu, i, label, value);
        core_fill(0, (int)y, 240, (int)MENU_LINE, sel ? CORE_SEL : CORE_BG);
        core_text(8, (int)y, 112, CORE_FONT_S, label, sel ? CORE_BG : CORE_MID, sel ? CORE_SEL : CORE_BG, 0);   /* (dark on
                                                                                    * the light highlight) */
        core_text(120, (int)y, 112, CORE_FONT_S, value, sel ? CORE_BG : CORE_TEXT, sel ? CORE_SEL : CORE_BG, 0);
    }
    menu_paint_req = 0;
}

static void menu_open(void)
{
    menu.open = 1;
    menu.item = 0;
    menu_last = 0xFFu;                        /* keys held now count once let go and pressed again */
    memset(&knobs, 0, sizeof knobs);          /* (turns meant for the Game Boy) */
    menu_paint_req = 1;
}

static void menu_close(void)
{
    menu.open = 0;
    menu_mask = menu_last;                    /* (the B that closed it, a direction still held) */
    memset(&knobs, 0, sizeof knobs);          /* (turns meant for the menu) */
    rows_resend(MENU_Y, 239u);                /* the picture under it again */
    st.pad_shown = (uint8_t)~st.pad;          /* (and the status bar, which its last line covers) */
    status_line();
    if (memcmp(&menu.set, &set_saved, sizeof set_saved))
        set_req = 1;                          /* into flash, in the next step */
}

static void menu_keys(uint8_t pad)             /* a frame's keys (active low) while the menu is open: presses only */
{
    uint8_t down = (uint8_t)~pad, now = (uint8_t)(down & ~menu_last), b;
    menu_last = down;
    for (b = 1; b; b = (uint8_t)(b << 1)) {
        if (!(now & b))
            continue;
        if (b == JOYPAD_B) {
            menu_close();
            return;
        }
        if (lsdj_menu_key(&menu, b))
            settings_apply();
        menu_paint_req = 1;
    }
}

/* ---- running ---- */
/* ---- flash: the save and the kit banks ---- */
static void lsdj_xfer_flash(void)              /* what `ble info` and the kit commands need to know */
{
    xfer.kit = kit_buf;
    xfer.kit_bank = kit_bank;
    xfer.nkit = nkit;
    xfer.kits = store.k < nkit ? store.k : nkit;
    xfer.saved = store.have && store.prog == prog;   /* (another program's save is not this one's) */
    xfer.prog = (uint8_t)(prog == LSDJ_PROG_DMGO);
    xfer.cap = lsdj_st_cap(store.k);
    xfer.used = store.have ? store.len : 0;
}

static void lsdj_banks(void)                   /* rom_tab: the ROM, and the K flash kit banks where it has none */
{
    uint32_t b, j;
    memcpy(rom_tab, cart.bank, sizeof rom_tab);
    for (b = 8, nkit = 0; prog == LSDJ_PROG_LSDJ && b < cart.nbanks && nkit < LSDJ_ST_KMAX; b++)   /* LSDj's kits:
                                                                                                * banks 8..26, 32.. */
        if ((b <= 26u || b >= 32u) && !cart.bank[b].n)
            kit_bank[nkit++] = (uint8_t)b;
    for (j = 0; j < store.k && j < nkit; j++) {   /* (store.k is the flash layout's: never changed here, even if a
                                                   * store made with another ROM has more banks than this one shows) */
        rom_tab[kit_bank[j]].p = core_flash_ptr(lsdj_st_kit_off(j));
        rom_tab[kit_bank[j]].n = LSDJ_BANK;
    }
    emu_hi_set(~0u);                          /* no bank cached */
    lsdj_xfer_flash();
}

/* ---- the announcement, as OPLBend Tracker's: a word in big letters in the middle (D-PAD, SAVED 12K...), over the
 * picture, in a framed box, for its time: its 5 x 7 font (lsdj_font.h) up to 4 times ---- */
#define ANN_MARGIN 10u
static char ann[24];
static uint8_t ann_on;
static uint32_t ann_ms, ann_for, ann_n, ann_e, ann_tx, ann_ty, ann_bx0, ann_bx1, ann_by0, ann_by1;

/* the box over `rows` screen rows from row y, each w pixels from screen column sx */
static void ann_paint(uint16_t *out, uint32_t y, uint32_t rows, uint32_t sx, uint32_t w)
{
    uint32_t r, x, xa, xb, pitch = (LSDJ_FONT_W + 1u) * ann_e;
    if (!ann_on || sx > ann_bx1 || sx + w <= ann_bx0 || y > ann_by1 || y + rows <= ann_by0)
        return;
    xa = sx > ann_bx0 ? sx : ann_bx0;
    xb = sx + w - 1u < ann_bx1 ? sx + w - 1u : ann_bx1;
    for (r = 0; r < rows; r++) {
        uint32_t ly = y + r;
        if (ly < ann_by0 || ly > ann_by1)
            continue;
        for (x = xa; x <= xb; x++) {
            uint16_t c = ly == ann_by0 || ly == ann_by1 || x == ann_bx0 || x == ann_bx1 ? pal[3] : pal[0];
            if (ly >= ann_ty && ly < ann_ty + LSDJ_FONT_H * ann_e && x >= ann_tx && x < ann_tx + ann_n * pitch) {
                uint32_t cx = (x - ann_tx) % pitch / ann_e, cy = (ly - ann_ty) / ann_e;
                uint8_t ch = (uint8_t)ann[(x - ann_tx) / pitch];
                if (cx < LSDJ_FONT_W && ch >= 0x20u && ch < 0x60u && (LSDJ_FONT[ch - 0x20u][cy] & (0x10u >> cx)))
                    c = pal[3];
            }
            out[r * w + x - sx] = c;
        }
    }
}

static void ann_show(const char *s, uint32_t ms)   /* at once (a save holds everything else for a second or two) */
{
    uint32_t n = 0, lw, h, y, w, rows;
    if (ann_on)
        rows_resend(ann_by0, ann_by1);        /* (the box before, if it was bigger) */
    while (s[n] && n < sizeof ann - 1u) {
        ann[n] = s[n];
        n++;
    }
    ann[n] = 0;
    ann_on = 0;
    if (!n)
        return;
    ann_n = n;
    ann_e = (240u - 2u * ANN_MARGIN - 4u) / ((LSDJ_FONT_W + 1u) * n);
    ann_e = ann_e > 4u ? 4u : ann_e ? ann_e : 1u;
    lw = n * (LSDJ_FONT_W + 1u) * ann_e - ann_e;
    h = LSDJ_FONT_H * ann_e;
    ann_tx = (240u - lw) / 2u;
    ann_ty = (240u - h) / 2u;
    ann_bx0 = ann_tx - ANN_MARGIN;
    ann_bx1 = ann_tx + lw + ANN_MARGIN - 1u;
    ann_by0 = ann_ty - ANN_MARGIN;
    ann_by1 = ann_ty + h + ANN_MARGIN - 1u;
    ann_on = 1;
    ann_ms = core_ms();
    ann_for = ms;
    rows_resend(ann_by0, ann_by1);            /* (the picture sent there from now on carries the box) */
    w = ann_bx1 - ann_bx0 + 1u;
    rows = (uint32_t)(sizeof pushbuf[0] / sizeof pushbuf[0][0]) / w;
    for (y = ann_by0; y <= ann_by1; y += rows) {
        uint32_t k = ann_by1 + 1u - y < rows ? ann_by1 + 1u - y : rows;
        ann_paint(pushbuf[buf_i], y, k, ann_bx0, w);
        core_lcd_blit((int)ann_bx0, (int)y, (int)w, (int)k, pushbuf[buf_i]);
        buf_i ^= 1u;
    }
}

static void lsdj_note_for(const char *s, uint32_t ms)   /* in the middle now, for ms */
{
    ann_show(s, ms);
    put_s(st.note, s);
    st.note_ms = core_ms();
    st.note_for = ms;
    keys_line();
}
static void lsdj_note(const char *s) { lsdj_note_for(s, 2000u); }

static int lsdj_save(void)                     /* the save RAM into flash: a second or two, nothing else runs */
{
    char b[12];
    int r;
    uint8_t was = store.prog;
    st.gap_off = 1;                           /* (silence, but not a gap of the Game Boy's) */
    lsdj_note("SAVING");
    store.prog = (uint8_t)prog;               /* the save in flash is this program's from now on */
    r = flash_on ? lsdj_store_save(&flash, &store, sram, save_n) : LSDJ_ST_FAIL;
    if (r == LSDJ_ST_FULL || !flash_on)       /* nothing written: the save in flash is still whose it was (after a
                                               * flash error the store was read again: it says so itself) */
        store.prog = was;
    if (r == LSDJ_ST_OK) {
        put_s(put_u(put_s(b, "SAVED "), (store.len + 1023u) / 1024u), "K");
        lsdj_note(b);
    } else {
        lsdj_note(r == LSDJ_ST_FULL ? "SAVE FULL" : "SAVE FAILED");
    }
    lsdj_xfer_flash();
    return r;
}

static void lsdj_flash_req(void)               /* the console queued it (lsdj_xfer.c): flash is written only here */
{
    int r = LSDJ_ST_FAIL;
    st.gap_off = 1;
    if (xfer.req == LSDJ_REQ_SAVE)
        r = lsdj_save();
    else if (!flash_on)
        r = LSDJ_ST_FAIL;
    else if (xfer.req == LSDJ_REQ_KIT)
        r = lsdj_store_kit_write(&flash, &store, xfer.req_arg, kit_buf);
    else if (xfer.req == LSDJ_REQ_KIT_ERASE)
        r = lsdj_store_kit_erase(&flash, &store, xfer.req_arg);
    else if (xfer.req == LSDJ_REQ_KITS && prog == LSDJ_PROG_LSDJ && store.prog == LSDJ_PROG_LSDJ)
        r = lsdj_store_kits(&flash, &store, xfer.req_arg, sram, save_n);   /* (LSDj's kits, with LSDj's save) */
    lsdj_banks();                             /* K may have changed */
    xfer.res = (uint8_t)r;
    xfer.req = LSDJ_REQ_NONE;
}

static void lsdj_restart(void)                 /* `ble go reset`: the Game Boy from its reset, the save RAM kept */
{
    emu_init(rom_tab, cart.nbanks, sram, fb);         /* (it ran before: the cartridge is LSDj's or DMGo's) */
    lsdj_steps_restart(&steps);
    emu.dirty = dirty;
    memset(shown, 0xFF, sizeof shown);        /* every line pair is sent again */
    memset(dirty, 1, sizeof dirty);
}

static void lsdj_step(void)
{
    uint32_t t0, us, n, notes;
    t0 = core_ticks();                        /* the longest time from one step to the next, in each second */
    us = (t0 - st.step_ticks) / CORE_TICKS_PER_US;
    st.step_ticks = t0;
    if (us > st.sec_pass_max)
        st.sec_pass_max = us;
    if (core_ms() - st.sec_ms >= 1000u) {
        st.sec_ms += 1000u;
        st.last_fps = st.frames - st.sec_frames;
        st.sec_frames = st.frames;
        st.last_under = ring.under - st.sec_under;
        st.sec_under = ring.under;
        if (st.last_under && !st.gap_off) {   /* the sound ran dry in this second, and nobody stopped the Game Boy */
            st.gaps++;
            st.pad_shown = (uint8_t)~st.pad;  /* redraw GAPS */
        }
        st.gap_off = 0;
        st.last_load = st.last_fps ? st.sec_us / st.last_fps * 100u / 16742u : 0;
        st.last_skipped = st.last_fps && emu.skipped >= st.sec_skipped ? (emu.skipped - st.sec_skipped) / st.last_fps : 0;
        st.sec_skipped = emu.skipped;         /* (it starts again with the Game Boy: `ble go reset`) */
        st.last_frame_max = st.sec_frame_max;
        st.last_pass_max = st.sec_pass_max;
        st.sec_frame_max = st.sec_pass_max = 0;
        st.sec_us = 0;
        if (st.sec_ms - st.win_ms >= 4000u) {
            st.last_speed = lsdj_speed(st.frames - st.win_frames, st.sec_ms - st.win_ms);
            st.last_fps10 = (st.frames - st.win_frames) * 10000u / (st.sec_ms - st.win_ms);
            st.win_ms = st.sec_ms;
            st.win_frames = st.frames;
        }
        status_line();
    }
    lsdj_xfer_tick(&xfer, core_ms());
    if (!dpad)                                /* GLO blinking in KEYBOARD */
        core_led_button(CORE_BTN_GLO, glo_led());
    if (st.note[0] && core_ms() - st.note_ms >= st.note_for)
        st.note[0] = 0;
    if (ann_on && core_ms() - ann_ms >= ann_for) {   /* the announcement gone: the picture under it again */
        ann_on = 0;
        rows_resend(ann_by0, ann_by1);
        if (menu.open)
            menu_paint_req = 1;
    }
    if (st.pad != st.pad_shown || xfer.held != st.held_shown)
        keys_line();
    if (xfer.req)
        lsdj_flash_req();
    if (set_req && !xfer.held) {              /* the menu closed with something changed: into flash */
        uint8_t b[2];
        set_req = 0;
        lsdj_set_pack(&menu.set, b);
        st.gap_off = 1;
        if (flash_on && lsdj_store_settings(&flash, &store, b) == LSDJ_ST_OK)
            set_saved = menu.set;
        else
            lsdj_note("SETTINGS FAILED");     /* (tried again when the menu next closes with a change) */
    }
    if (menu.open && menu_paint_req && core_lcd_idle())   /* (also while the host holds the Game Boy) */
        menu_paint();
    if (!(core_buttons() & core_rec_bit())) { /* REC held for a second: save (once a press; not under the host) */
        st.rec_ms = 0;
        st.rec_done = 0;
    } else if (!st.rec_ms) {
        st.rec_ms = core_ms() | 1u;
    } else if (!st.rec_done && !xfer.held && core_ms() - st.rec_ms >= 1000u) {
        st.rec_done = 1;
        lsdj_save();
    }
    if (xfer.held) {
        st.gap_off = 1;
        return;                               /* the host reads or writes the save: no Game Boy frame */
    }
    if (xfer.reset_req) {                     /* the save changed under LSDj: it starts again and reads it */
        xfer.reset_req = 0;
        lsdj_restart();
    }
    if (lsdj_ring_free(&ring) < LSDJ_FRAME_SAMPLES) {     /* ahead of the audio: wait for it */
        st.waits++;
        return;
    }
    st.pot = core_master_pot();
    st.master_q12 = lsdj_master(&st.knob, st.pot);
    /* A pass of the core's main loop lasts 15 ms or more (core_main.c), and a Game Boy frame is 16.7 ms of sound:
     * one frame a pass makes at most 12 % more sound than is played, and after a slow pass the ring would stay
     * low for a long time. So with the ring under half, a second frame runs in the same pass. (Measured on the
     * FM-1, 2026-10-13: 67 passes a second with nothing playing.) */
    for (n = 0; n < 2u; n++) {
        t0 = core_ticks();
        st.buttons = core_buttons();
        st.notes = core_notes();
        notes_mask &= st.notes;               /* note keys held when GLO changed the mode: nothing until let go */
        notes = st.notes & ~notes_mask;
        if (dpad)                             /* the cross on the note keys (not also on a button) repeats as OPLBend's */
            st.pad = lsdj_rep_frame(&rep, lsdj_pad_dpad(st.buttons, notes, pad_map),
                                    (uint8_t)(~lsdj_pad_dpad(0, notes, pad_map) & lsdj_pad_ab(st.buttons, 0, pad_map, 0, 0)));
        else
            st.pad = lsdj_pad_ab(st.buttons, notes, pad_map, 0, 0);
        st.pad &= (uint8_t)~lsdj_knobs_frame(&knobs);
        if (menu.open || dpad) {              /* the menu takes the keys; in D-PAD the note keys are the pad's */
            if (with_steps && steps.busy)     /* a DMGo step half done: its cursor back now (B stays up) */
                lsdj_steps_frame(&steps, emu.gb.wram, 0xFF);
            if (with_steps) {
                lsdj_steps_keys(&steps, notes);   /* (white keys held now are not pressed when it closes) */
                steps.n = 0;
            }
        }
        if (menu.open) {                      /* the Game Boy gets no key, and plays on */
            menu_keys(st.pad);
            st.pad = 0xFF;
        } else {
            menu_mask &= (uint8_t)~st.pad;    /* a key held since the menu closed: up for the Game Boy until let go */
            st.pad |= menu_mask;
            if (with_steps && !dpad) {        /* DMGo's step keys: its cursor on the step and B, for a frame */
                lsdj_steps_keys(&steps, notes);
                st.pad = lsdj_steps_frame(&steps, emu.gb.wram, st.pad);
            }
        }
        if (xfer.key_n) {                     /* the console's `ble key` */
            xfer.key_n--;
            st.pad &= (uint8_t)~xfer.key;
        }
        emu_frame(st.pad);
        lsdj_ring_push(&ring, emu.snd, LSDJ_FRAME_SAMPLES);
        us = (core_ticks() - t0) / CORE_TICKS_PER_US;
        st.frames++;
        st.frame_us_sum += us;
        st.sec_us += us;
        if (us > st.frame_max_us)
            st.frame_max_us = us;
        if (us > st.sec_frame_max)
            st.sec_frame_max = us;
        if (lsdj_ring_fill(&ring) >= RING_LOW)
            break;
        st.doubles += n == 0;
    }
}

/* changed line pairs to the screen, round robin, never waiting: while the last transfer runs, nothing; else one
 * transfer of the next changed line pair and up to PUSH_MAX - 1 changed ones right below it (their spans' union),
 * which goes out over SPI while the next Game Boy frame runs */
static void lsdj_push(void)
{
    uint32_t k, g0, n, rows, gend, x0, x1, a, b, r, w;
    if (!core_lcd_idle())
        return;
    if (border_req) {                         /* a border, in the theme's lightest shade: one transfer */
        uint32_t i, top = border_req & 1u;
        for (i = 0; i < 240u * BORDER; i++)
            pushbuf[buf_i][i] = pal[0];
        core_lcd_blit(0, top ? 0 : (int)(240u - BORDER), 240, (int)BORDER, pushbuf[buf_i]);
        buf_i ^= 1u;
        border_req &= (uint8_t)(top ? ~1u : ~2u);
        return;
    }
    zoom_follow();
    gend = menu.open ? menu_group() : g_n;    /* (under the menu: later) */
    for (k = 0; k < g_n; k++) {               /* only rows marked changed are compared */
        g0 = group;
        group = group + 1u >= g_n ? 0 : group + 1u;
        if (g0 >= gend || !group_dirty(g0))
            continue;
        if (group_span(g0, &x0, &x1))
            break;
        memset(dirty + g0 * g_lines, 0, g_lines);   /* changed back: nothing to send */
    }
    if (k == g_n)
        return;                               /* the screen shows the frame */
    rows = g_h(g0);                           /* the groups right below it on screen too, PUSH_MAX x 3 rows at most */
    for (n = 1; g0 + n < gend && g_h(g0 + n) && g_y(g0 + n) == g_y(g0) + rows && rows + g_h(g0 + n) <= PUSH_MAX * 3u &&
                group_dirty(g0 + n) && group_span(g0 + n, &a, &b); n++) {
        x0 = a < x0 ? a : x0;
        x1 = b > x1 ? b : x1;
        rows += g_h(g0 + n);
    }
    w = gx(x1) - gx(x0);
    for (k = 0, r = 0; k < n; r += g_h(g0 + k), k++) {
        uint16_t *o = pushbuf[buf_i] + r * w;
        if (zoomed)
            lsdj_scale_zoom(o, fb[8u * (g0 + k)], x0, x1, pal, g_h(g0 + k));
        else
            lsdj_scale(o, fb[2u * (g0 + k)], x0, x1, pal);
        ann_paint(o, g_y(g0 + k), g_h(g0 + k), gx(x0), w);
    }
    core_lcd_blit((int)gx(x0), (int)g_y(g0), (int)w, (int)rows, pushbuf[buf_i]);
    buf_i ^= 1u;
    for (r = g_lines * g0; r < g_lines * (g0 + n); r++) {
        memcpy(&shown[r][x0], &fb[r][x0], x1 - x0);
        dirty[r] = 0;                         /* the span covers all of these rows' changes */
    }
    group = g0 + n >= g_n ? 0 : g0 + n;
    st.pushes++;
}

static __attribute__((noinline)) void lsdj_start(void)
{
    int r = lsdj_cart_read(core_cart_end(), core_cart_room(), &cart);
    if (r) {                                  /* no ROM in this firmware (or not a whole one): nothing runs */
        st.error = r == LSDJ_CART_NONE ? -3 : -4;
        core_fill(0, 0, 240, 240, CORE_BG);
        core_text(0, 100, 240, CORE_FONT_M, r == LSDJ_CART_NONE ? "NO LSDJ ROM" : "LSDJ ROM DAMAGED", CORE_TEXT,
                  CORE_BG, 1);
        core_text(0, 126, 240, CORE_FONT_S, r == LSDJ_CART_NONE ? "PUT YOURS IN WITH THE INSTALL PAGE" :
                  "INSTALL IT AGAIN", CORE_MID, CORE_BG, 1);
        return;
    }
    {
        uint8_t h[LSDJ_HDR];
        lsdj_header(cart.bank, cart.nbanks, h);
        prog = lsdj_prog(h);
        save_n = lsdj_prog_save(prog);
        with_steps = (uint8_t)lsdj_prog_steps(h);
    }
    if (prog == LSDJ_PROG_NONE) {
        st.error = -1;
        core_fill(0, 0, 240, 240, CORE_BG);
        core_text(0, 110, 240, CORE_FONT_M, "NOT LSDJ OR DMGO", CORE_TEXT, CORE_BG, 1);
        return;
    }
    r = -1;
    flash_on = core_flash_ptr(LSDJ_ST_BASE) != 0;
    memset(&menu, 0, sizeof menu);
    {
        static const uint8_t none[2] = {0, 0};   /* the menu's defaults: DMG green, zoom on, no status bar, all on */
        lsdj_set_unpack(&menu.set, none);
    }
    if (flash_on) {                           /* (reading only: flash is written on REC or a console request) */
        lsdj_store_scan(&flash_ro, &store);
        lsdj_set_unpack(&menu.set, store.set);
        if (store.have && store.prog == prog) /* the other program's save stays, unread, until REC saves over it */
            r = lsdj_store_load(&flash_ro, &store, sram, save_n);
    }
    if (r)                                    /* no save in flash: the start-up save, as built */
        r = lsdj_unpack(sram, save_n, cart.save, cart.save_len);
    lsdj_banks();
    core_fill(0, 0, 240, 240, CORE_BG);
    if (r) {
        st.error = -2;
        core_text(0, 110, 240, CORE_FONT_M, "START SAVE DAMAGED", CORE_TEXT, CORE_BG, 1);
        return;
    }
    memset(fb, 0, sizeof fb);
    set_saved = menu.set;
    set_req = 0;
    bar_shown = 0;
    emu_mode = (EMU_FAST_ROM | EMU_FRAME_SKIP | EMU_WAIT_SKIP | EMU_SND_FINE) & ~(uint32_t)menu.set.off;
    theme_apply();                            /* (no shade sent yet: every line pair is sent once) */
    geometry_set();                           /* (the screen was cleared above) */
    lsdj_ring_init(&ring, ring_buf, RING_N);
    r = emu_init(rom_tab, cart.nbanks, sram, fb);
    emu.dirty = dirty;
    if (r) {
        st.error = r;
        core_text(0, 110, 240, CORE_FONT_M, "NOT LSDJ OR DMGO", CORE_TEXT, CORE_BG, 1);
        return;
    }
    for (r = 0; r < 8; r++)
        pad_map[r] = KEYS[r] < 0 ? 0 : core_button_bit(KEYS[r]);
    dpad = 1;                                 /* D-PAD at start (GLO: KEYBOARD) */
    notes_mask = core_notes();                /* (keys held at start: nothing until let go) */
    st.start_ms = st.sec_ms = st.win_ms = core_ms();
    st.step_ticks = core_ticks();
    st.pad = 0xFF;
    st.pad_shown = 0;                         /* draw the keys part once */
    st.gaps = 0;
    st.gap_off = 1;                           /* (the ring is empty until the first frame: not a gap) */
    memset(&steps, 0, sizeof steps);
    lsdj_steps_restart(&steps);
    if (store.have && store.prog != prog)     /* the FM-1 has the other program's save: say why it is not here */
        lsdj_note_for(store.prog == LSDJ_PROG_DMGO ? "DMGO SAVE KEPT" : "LSDJ SAVE KEPT", 8000u);
    draw = lsdj_push;
    run = lsdj_step;
    xfer.started = 1;
    dpad_leds();
}

/* ---- the core's hooks (named by the BLE test; see the top) ---- */
void ble_key(uint32_t pressed_buttons, int32_t select_steps)
{
    int32_t algo = core_enc_steps(CORE_ENC_ALGO), preset = core_enc_steps(CORE_ENC_PRESET);   /* (always taken) */
    if (!run) {                               /* (called every pass: the start screen's time runs out here) */
        if (!st.error && ((pressed_buttons & core_play_bit()) || (splash_done && core_ms() - splash_ms >= SPLASH_MS)))
            lsdj_start();
        return;
    }
    if (xfer.held) {                          /* the host has the Game Boy: turns are not kept for later */
        memset(&knobs, 0, sizeof knobs);
    } else {
        lsdj_knobs_add(&knobs, LSDJ_AX_LR, select_steps);
        lsdj_knobs_add(&knobs, LSDJ_AX_UD, algo);
        if (prog == LSDJ_PROG_LSDJ) {         /* LSDj's screen shortcuts */
            lsdj_knobs_add(&knobs, LSDJ_AX_NAV_LR, preset);
            if (pressed_buttons & core_button_bit(CORE_BTN_OCTUP))
                lsdj_knobs_add(&knobs, LSDJ_AX_NAV_UD, -1);
            if (pressed_buttons & core_button_bit(CORE_BTN_OCTDN))
                lsdj_knobs_add(&knobs, LSDJ_AX_NAV_UD, 1);
        }
    }
    if (pressed_buttons & core_button_bit(CORE_BTN_GLO)) {        /* GLO: KEYBOARD <-> D-PAD */
        dpad ^= 1u;
        notes_mask = core_notes();
        lsdj_note_for(dpad ? "D-PAD" : "KEYBOARD", 1000u);   /* (as OPLBend's: a second) */
        dpad_leds();
    }
    if (pressed_buttons & core_button_bit(CORE_BTN_ENV)) {        /* ENV: the menu (the emulator's options are in it) */
        if (menu.open)
            menu_close();
        else
            menu_open();
    }
}

void ble_service(void)
{
    void (*f)(void) = run;
    if (f)
        f();
}

void ble_draw(void)
{
    void (*f)(void) = draw;
    if (f) {
        uint32_t t0 = core_ticks();
        f();
        st.push_ticks += core_ticks() - t0;
        st.push_calls++;
        return;
    }
    if (splash_done || st.error)
        return;
    splash_done = 1;
    splash_ms = core_ms();
    /* (before PLAY nothing reads the cartridge: gate 4. So both programs' keys) */
    core_text(0, 128, 240, CORE_FONT_S, "STARTS IN 3 S (PLAY: AT ONCE)", CORE_THEME, CORE_BG, 1);
    core_text(0, 144, 240, CORE_FONT_S, "GLO: KEYBOARD MODE   ENV: MENU", CORE_MID, CORE_BG, 1);
    core_text(0, 160, 240, CORE_FONT_S, "LEFT RIGHT: SELECT KNOB", CORE_MID, CORE_BG, 1);
    core_text(0, 176, 240, CORE_FONT_S, "UP DOWN: ALGORITHM KNOB  SEL: SEQ", CORE_MID, CORE_BG, 1);
    core_text(0, 192, 240, CORE_FONT_S, "D-PAD  A: F#5  B: F5  ARROWS: F3-A3", CORE_MID, CORE_BG, 1);
    core_text(0, 208, 240, CORE_FONT_S, "SCREENS: PRESETS, OCT- OCT+", CORE_MID, CORE_BG, 1);
    core_text(0, 224, 240, CORE_FONT_S, "DMGO STEPS: WHITE KEYS (KEYBOARD)", CORE_MID, CORE_BG, 1);
}

static void lsdj_console(const char *args)     /* the words after `ble` on the console line: a transfer command */
{
    xfer.sram = sram;
    xfer.size = save_n;
    xfer.rom = rom_tab;                     /* (PLAY reads the block and the store: no bank before it) */
    xfer.nbanks = run ? cart.nbanks : 0;
    xfer.mode = (uint8_t)emu_mode;
    lsdj_xfer_cmd(&xfer, args, core_ms(), xfer_out);
    if (xfer.mode_set) {                      /* `ble mode M`: as the menu's emulator options (not kept in flash) */
        xfer.mode_set = 0;
        emu_mode = xfer.mode;
        menu.set.off = (uint8_t)(~emu_mode & 15u);
        st.pad_shown = (uint8_t)~st.pad;
    }
    core_con_puts(xfer_out);
    core_con_puts("\r\n");
}

void ble_status(void)                          /* the console's `ble`: the status; `ble WORDS`: a transfer command */
{
    char b[128], *p = b;
    const char *a = core_con_line();          /* (console.c is not ours to change: gate 2b; it calls this for `ble`) */
    while (*a == ' ')
        a++;
    if (a[0] == 'b' && a[1] == 'l' && a[2] == 'e')
        a += 3;
    while (*a == ' ')
        a++;
    if (*a) {
        lsdj_console(a);
        return;
    }
    p = put_s(p, run ? "lsdj running " : "lsdj stopped ");
    p = put_s(p, "error ");
    if (st.error < 0) {
        p = put_s(p, "-");
        p = put_u(p, (uint32_t)-st.error);
    } else {
        p = put_u(p, (uint32_t)st.error);
    }
    p = put_s(p, " emu_errors ");
    p = put_u(p, emu.errors);
    put_s(p, "\r\n");
    core_con_puts(b);
    p = put_s(b, "frames ");
    p = put_u(p, st.frames);
    p = put_s(p, " avg_us ");
    p = put_u(p, st.frames ? st.frame_us_sum / st.frames : 0);
    p = put_s(p, " max_us ");
    p = put_u(p, st.frame_max_us);
    p = put_s(p, " fps ");
    p = put_u(p, st.last_fps);
    p = put_s(p, " waits ");
    p = put_u(p, st.waits);
    p = put_s(p, " doubles ");                        /* passes with a second frame */
    p = put_u(p, st.doubles);
    put_s(p, "\r\n");
    core_con_puts(b);
    p = put_s(b, "keys buttons 0x");
    p = put_hex(p, st.buttons);
    p = put_s(p, " notes 0x");
    p = put_hex(p, st.notes);
    p = put_s(p, " play 0x");
    p = put_hex(p, core_play_bit());
    p = put_s(p, " rec 0x");
    p = put_hex(p, core_rec_bit());
    p = put_s(p, " pad 0x");
    p = put_hex(p, st.pad);
    put_s(p, "\r\n");
    core_con_puts(b);
    p = put_s(b, "master pot ");
    p = put_u(p, st.pot < 0 ? 9999u : (uint32_t)st.pot);
    p = put_s(p, " gain_q12 ");
    p = put_u(p, st.master_q12);
    put_s(p, "\r\n");
    core_con_puts(b);
    p = put_s(b, "mode ");
    p = put_u(p, emu_mode);
    p = put_s(p, " (1 fast rom, 2 frame skip, 4 wait skip, 8 fine sound) load% ");
    p = put_u(p, st.last_load);
    put_s(p, "\r\n");
    core_con_puts(b);
    p = put_s(b, "gap seconds ");
    p = put_u(p, st.gaps);
    p = put_s(p, " wait skipped ");                   /* instructions of LSDj's wait loop not run, a frame */
    p = put_u(p, st.last_skipped);
    p = put_s(p, " sec_frame_max_us ");
    p = put_u(p, st.last_frame_max);
    p = put_s(p, " sec_pass_max_us ");
    p = put_u(p, st.last_pass_max);
    put_s(p, "\r\n");
    core_con_puts(b);
    p = put_s(b, "audio gaps ");
    p = put_u(p, ring.under);
    p = put_s(p, " fill ");
    p = put_u(p, lsdj_ring_fill(&ring));
    p = put_s(p, " pushes ");
    p = put_u(p, st.pushes);
    p = put_s(p, " push_avg_us ");
    p = put_u(p, st.push_calls ? st.push_ticks / CORE_TICKS_PER_US / st.push_calls : 0);
    p = put_s(p, " s ");
    p = put_u(p, (core_ms() - st.start_ms) / 1000u);
    put_s(p, "\r\n");
    core_con_puts(b);
}

int ble_started(void) { return run != 0; }
