# Splash animation generation

## Splash animation generation

### `logo/gen_logo_icon.py`

`gen_logo_icon.py`:

Render the BWBots logo as a small transparent PNG for a dark top bar.

Produces the dark-background variant used by the splash animation: the logo's black half-disc is drawn as a white outline (it would vanish against the dark bar), the white half is plain fill, and the BW pill comes from the SVG. The background is transparent so the icon sits directly on the bar.

Requires: inkscape on PATH. Run inside the project venv (numpy, Pillow).

```
Usage:
    logo/gen_logo_icon.py                  # 24 px tall -> logo/bwbots_logo_dark_h24.png
    logo/gen_logo_icon.py --height 48
```

### `logo/gen_splash_animation.py`

`gen_splash_animation.py`:

Regenerate the BWBots splash animation at high resolution from the logo SVG.

The original 160x128 bwbots_splash.gif was authored at low resolution. This script rebuilds the same animation from the vector logo (logo/bwbots_logo.svg) at the robot UI's screen size (1024x600 by default, full-screen on the Jetson LCD):

```
  frame 0      black
  frames 1-7   the two half-discs slide in from the screen edges and meet
  frames 8-16  the assembled disc spins one full clockwise turn, decelerating
  frames 17-26 the halves fly apart, overshoot, and settle; the BW pill wipes
               open from the center on top of them (frames 17-20)
```

The per-frame motion table below was measured from the original GIF via connected-component analysis (positions normalized by the disc radius). Artwork proportions come from the SVG, which differs slightly from the hand-made GIF: the GIF's pill is ~8% larger and its final gap ~30% wider than the vector logo. The SVG is the true mark, so the final composition uses SVG proportions and the split trajectory is rescaled to land there.

Rendering: the half-discs are plain semicircles drawn supersampled with PIL (the SVG halves have notches that are always hidden behind the pill). The pill + "BW" text is rasterized from the SVG via the inkscape CLI (glyphs are path outlines, no fonts needed). On the black background the black half is drawn as a white outline and the white half as plain fill, exactly like the original GIF.

The disc size tracks the output height (same proportion as the reference GIF). When the canvas is wider than the reference's 5:4 aspect, the slide-in stage is rescaled so the halves still enter from the physical screen edges instead of materializing mid-screen.

Verification: every rendered frame is center-cropped to the reference aspect, downscaled to 160x128, and diffed against the original GIF frame. A contact sheet (original / rendered / diff) and per-frame mean absolute error are written next to the output. On canvases wider than 5:4 the slide-in frames legitimately differ (longer runway), so expect elevated MAE there.

Requires: inkscape on PATH. Run inside the project venv (numpy, Pillow).

```
Usage:
    logo/gen_splash_animation.py                    # 1024x600 (robot LCD)
    logo/gen_splash_animation.py --size 480x384 --fps-mult 1 --no-verify
```

- `REF_SIZE`: Reference GIF geometry (the motion table is normalized against these).
- `HOLD_LAST_MS`: Hold the assembled logo before the GIF ends/loops so playback doesn't cut off abruptly (the UI pauses the gif during this window, then fades to the app).
- `RADIUS_FRAC`: Disc radius as a fraction of output height (from the reference GIF: disc radius 30 on a 128-tall canvas, disc centered).
- `SVG_VIEWBOX_W`: SVG geometry, in path coordinates of bwbots_logo.svg (before the group's 0.2645833 scale). Used to size/position the pill and the final layout. black half-disc vertical extent.
- `FINAL_HALF_GAP`: Final logo half-gap (flat edge of each half-disc to logo center), fraction of r. From the SVG: flats at x=203.5 / 277.2, gap 73.7 units on a 136.5 radius.
- `GIF_FINAL_HALF_GAP`: The GIF's final half-gap is wider (10.5 px / 30 px); the measured split offsets below are rescaled so the same trajectory settles at the SVG's layout.
- `gen_splash_animation.py`: Per-frame motion, measured from bwbots_splash_160x128.gif. left_off / right_off: signed offset of each half-disc's flat edge from the disc center, in units of disc radius (left half sits left of its flat edge). angle: clockwise rotation of the assembled disc, degrees (0 = white half right). pill: revealed width fraction of the pill (center-out wipe).

`build_keyframes`:

Adapt the measured table to the output canvas.

Slide-in offsets (frames 0-6) are scaled so the halves enter from the actual canvas edge (the reference table assumed a 5:4 canvas, 2.667 radii wide). The first visible frame keeps the reference's look: the flat edge 0.12 r inside the screen edge. Split offsets (frames 17+) are scaled to settle at the SVG layout.

- `main`: Expected: MAE 2-5 on spin frames, up to ~22 on split frames (intentional SVG-true pill artwork / gap vs the hand-made GIF) and elevated on slide frames when the canvas is wider than 5:4 (longer slide-in runway).
