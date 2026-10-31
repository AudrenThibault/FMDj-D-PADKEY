# FMDj: controls

| Game Boy | FM-1 |
|---|---|
| LEFT / RIGHT | SELECT knob (a step = a press; clockwise = RIGHT). Also the HOME / ARP buttons, which can be held. |
| UP / DOWN | ALGORITHM knob (clockwise = DOWN). Also the SCL / SAVE buttons, which can be held. |
| A, B | the keyboard in D-PAD mode (below) |
| START | PLAY |
| SELECT | SEQ |

Shortcuts for LSDj's screens:

| LSDj | FM-1 |
|---|---|
| SELECT + LEFT / RIGHT | PRESETS knob |
| SELECT + UP | OCT+ |
| SELECT + DOWN | OCT- |

Editing as in LSDj: hold A (F#5, D-PAD mode) and turn SELECT to change a value by one, ALGORITHM by an octave or 0x10.

The start screen shows the keys for 3 s, then LSDj starts by itself (PLAY starts it at once).

The FM-1's own: MASTER is the volume; OCT- and OCT+ held together for 5 s enter UBOOT.

## D-PAD mode (GLO)

**GLO** switches between **KEYBOARD** and **D-PAD**. In KEYBOARD the keyboard is a keyboard: no Game Boy key on it
(in DMGo the white keys are its steps). In D-PAD it is a Game Boy pad: the cross on the left, A to the right of B, as
on a Game Boy. The name of the mode shows in big letters in the middle of the screen for a
second. The knobs and buttons above still work in both modes. Each power-on starts in D-PAD.
The LEDs show the mode as in OPLBend Tracker: in D-PAD GLO is lit and only the pad's keys glow; in KEYBOARD every key
glows and GLO blinks.

| Game Boy | FM-1 keyboard in D-PAD mode |
|---|---|
| LEFT | F3 (the leftmost key) |
| UP | F#3 |
| DOWN | G3 |
| RIGHT | A3 |
| SELECT | B3 and C4 (left hand), D5 (right hand) |
| START | D#5 (and PLAY) |
| B | E5 and F5 |
| A | F#5 |

A cross key held moves once, then after a third of a second keeps moving, 15 steps a second (LSDj's own, quicker
repeat does not start; LSDj misses a press shorter than two frames, so not OPLBend Tracker's 30). In DMGo, the white keys are not its steps in D-PAD mode. A key held while GLO is pressed does nothing until it is
let go.

## The menu (ENV)

**ENV** opens the menu over the lower half of the picture; ENV (or B) closes it. The Game Boy plays on while it is
open, and the keys are the menu's: UP / DOWN (the ALGORITHM knob, SCL / SAVE) choose a line, LEFT / RIGHT (the
SELECT knob, HOME / ARP) or A change it. What changed is kept in the FM-1's flash when the menu closes, and comes
back at the next PLAY.

| Line | Values (first: as it starts) | What it is |
|---|---|---|
| SCREEN | DMG, POCKET, LIGHT, SGB, DARK | the screen's colours: the first Game Boy's green, the Pocket's grey, the Light's backlit teal, the Super Game Boy's 1-A, light on black |
| ZOOM | ON, OFF | as OPLBend Tracker's 4 COL: LSDj's left 15 columns, its two top lines, then 10 of its rows (00 to 09 on SONG) in big; the view moves a row at a time to keep the cursor in it (its right panel, the channel, tempo and screen map, is cut); OFF: the whole Game Boy screen |
| STATUS BAR | OFF, ON | the line under the picture (below) |
| DRAW | EVERY 2ND, EVERY | every frame drawn: a smoother screen, a higher CPU |
| ROM READ | FAST, PLAIN | the plain ROM read |
| WAIT LOOP | SKIP, RUN | LSDj's wait loop run instruction by instruction: the CPU without the saving |
| SOUND | IN FRAME, PER FRAME | the sound made once per Game Boy frame, as before: notes up to 17 ms off their beat |

The last four were the FX, LFO, EDIT and ENV buttons; FX, LFO and EDIT do nothing now, ENV opens this menu.

Saving the settings when the menu closes takes a moment of flash writing: the sound stops for a fraction of a
second (only when something changed). The very first time anything of FMDj's is written to the FM-1's flash (a
save, kits, or these settings), the user-sample headers Hortator keeps there are cleared, as its samples' area is
FMDj's from then on.

## The status bar

Off at first (the menu's STATUS BAR): the picture is then in the middle of the screen, with a border above and below
in the theme's lightest shade, so the whole screen is the Game Boy's. With the status bar on, the picture moves up
and the bar is under it.

With it on, left: **FPS** is the Game Boy's frames a second (59.7 is real time). **CPU**
is how much of its time the FM-1 needs for the Game Boy: under 100 % it keeps up, and the lower the more is spare.

Right, with no key held: **GAPS** counts the seconds so far in which the sound ran dry, because the FM-1 was too
slow just then. A gap is heard as a short stutter. While the number stands still, nothing was lost. After it, an
emulator option that is off is named: FULL (DRAW), SLOW (ROM READ), SPIN (WAIT LOOP), OLD (SOUND).

**REC held for a second saves to the FM-1's flash** (under the picture, with or without the status bar: SAVING,
then SAVED and the size; SAVE FULL if it does not fit). LSDj loads that save when it starts.

A knob turned fast is followed at up to 30 presses a second; no more than 16 steps wait. A screen shortcut (the
PRESETS knob, OCT- / OCT+) holds SELECT two frames before its direction: LSDj's PROJECT screen takes nothing less.
