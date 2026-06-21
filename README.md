# Dynamic Autocrop

An OBS Studio plugin that crops the black (or near-black) border produced by retro upscalers and line-doublers (RetroTink, OSSC, Morph, and similar devices) running in a fixed-canvas "scale under" style mode, then stretches the active picture to fill a configured output resolution -- all in real time, automatically.

## The problem it fixes

Many retro upscalers and line-doublers, when set to output a fixed-size canvas rather than the console's native resolution (RetroTink calls this mode "Scale: Under"; other devices use their own names for the same idea), produce a fixed canvas (e.g. 1920x1080) with the actual active picture sitting somewhere inside it, surrounded by a black border whose size depends on the source resolution being scaled. That border size isn't constant -- different consoles, and even different games on the same console, can produce different border thicknesses. OBS's own Crop/Pad filter can crop a fixed number of pixels, but can't adapt automatically when the border changes. This plugin detects the actual border on every frame and crops to it dynamically, so switching games or consoles doesn't require manually re-measuring and re-entering crop values.

## How it works

### Detection

The filter periodically renders the source into a small 1280x720 texture (16:9, matching the canvas most capture/scaler setups actually deliver, so detection precision is balanced across both axes rather than skewed toward one) and reads the pixels back to the CPU:

1. **Brightness / Max Black % gates** -- skip the pass entirely if the frame is too dark or too uniformly black to give a trustworthy reading (loading screens, boot logos, fades).
2. **Per-row/column content scan** -- a row or column only counts as "content" once a meaningful fraction of its pixels exceed Black Threshold, not just a single stray pixel, so an isolated noise spike or hot pixel can't fool detection on its own.

Horizontal/Vertical Trim and Max Crop X/Y aren't part of detection itself -- they're applied afterward, every frame, regardless of how often detection runs.

### Post-processing

Max Crop X/Y and Horizontal/Vertical Trim are both applied fresh on every single rendered frame:

1. **Max Crop X/Y** is enforced first -- a hard ceiling, pulling the crop back outward (expanding from wherever the detected border already sits, not forced to an even split) if detection wants to remove more than the configured limit.
2. **Horizontal/Vertical Trim** is applied last, independently per axis. Nothing overrides it afterward.

Because of this, **Max Crop X/Y and Horizontal/Vertical Trim take effect the instant you change them** -- even while Freeze Crop is on, or while detection itself is paused by Min Brightness / Max Black % on a dark scene.

### Commit

A newly detected crop has to match what was found on the previous scan, within 1.5% on all four edges, for `Debounce` consecutive scans, before it actually applies -- this stops a single bad or transitional frame from snapping the picture to a wrong crop. Two things skip this and apply immediately:

- The very first detection, right after the filter is added or the resolution changes.
- A 15-second safety timeout: if nothing has applied in 15 real seconds despite repeatedly trying, the next attempt applies regardless, so a noisy or drifting source can't get stuck indefinitely.

Everything else -- changing a setting, pressing Recalculate Crop Now, or the first scan after a dark/black gap -- still goes through that same matching check, just faster: it re-scans every 0.1s instead of waiting for the full Recalc Interval, so it typically settles within a fraction of a second.

If **Ignore Small Changes** is enabled, a newly confirmed crop is also compared against what's currently showing -- if every edge differs by less than Min Change %, it's skipped and the current crop is left exactly as-is, rather than applying a change too small to matter.

### Output

The final crop is applied via a GPU shader (whichever Scale Filter is selected), with a tiny built-in safety margin at the pixel level so a thin line of border colour can never bleed into the edge of the picture.

## Verification

Every OBS API used here was checked against the real OBS 31.0.0 source (`obsproject/obs-studio` tag `31.0.0`) rather than assumed -- including a direct symbol-by-symbol cross-check between every shader uniform declared and every uniform looked up on the C side, confirming neither has an orphaned or missing entry. The plugin compiles cleanly against those real headers, and the shader source is verified brace/paren-balanced as part of the same check.

## Project layout

```
obs-dynamic-autocrop/
  CMakeLists.txt       -- standalone, out-of-tree build
  build.bat            -- elevates, then runs build.ps1 (recommended entry point)
  build.ps1            -- Windows build script (see below)
  src/
    dynamic_autocrop_filter.c -- the filter itself (detection, post-processing, shader)
  data/
    locale/en-US.ini    -- module display name
```

## Settings

| Setting | What it does |
|---|---|
| **Output Width / Height** | The crop is stretched to fill exactly this resolution. `0` (default) automatically matches your OBS canvas resolution -- recommended, since it lets this filter do the full crop+resize in a single pass instead of OBS having to resize again afterward. Set both explicitly to override. |
| **Scale Filter** | `Linear` (smooth), `Point` (crisp nearest-neighbour, best at integer scales), `Sharp Bilinear` (nearest-neighbour grid with a thin anti-aliased blend -- common retro-upscaler look), or `Lanczos` (default, highest quality, heaviest -- 36 samples/pixel). |
| **Black Threshold** | How dark a pixel must be to count as border. Raise it for noisy composite/S-Video captures where "black" isn't truly 0. Default 50. |
| **Horizontal Trim %** | Extra inward trim from the left and right edges, beyond the detected border. Use for coloured garbage pixels sitting just outside the active picture (Mega Drive/Genesis dots, etc). Defaults to 0.5% rather than 0% -- this is what prevents a thin leftover sliver of border colour at the edge, so lowering it trades that protection away. |
| **Vertical Trim %** | Same as Horizontal Trim, for the top and bottom edges. Set independently. Defaults to 0.5% for the same reason. |
| **Recalc Interval (s)** | How often the filter re-scans for borders. Default 1.0s. |
| **Min Brightness %** | Skip re-scanning frames darker than this (average brightness across the whole frame). Default 15%. |
| **Max Black %** | Skip re-scanning if more of the frame than this is near-black. Catches dark scenes with a bright HUD that Min Brightness alone would miss. Default 60%. |
| **Debounce** | Stable samples required before applying a *changed* crop. The very first detection always applies immediately. Default 4. |
| **Max Crop X / Y %** | Hard ceiling on how much of the frame can be cropped away per axis, regardless of what detection finds. Protects genuine in-game black bars from being mistaken for scaler border and over-cropped -- X protects pillarboxing (e.g. TATE-mode vertical shooters like Ikaruga), Y protects letterboxing (e.g. title screens like Metroid Prime). Default 60% / 20%. |
| **Ignore Small Changes** | Skip applying a new crop if it's only marginally different from the current one -- helps avoid tiny adjustments caused by analog noise. On by default. |
| **Min Change %** | Only visible when Ignore Small Changes is on. How different (per edge) a new crop must be before it's treated as real rather than noise. Default 3%. |
| **Freeze Crop** | Stop re-scanning and hold the current crop exactly as-is. Useful as a workaround for games that otherwise confuse detection. |
| **Recalculate Crop Now** | Button, visible only while Freeze Crop is on -- forces one fresh re-scan on demand. |

## Requirements

- **OBS Studio 28.0** or later
- **CMake 3.16+**
- MSVC 2019+ (Visual Studio Build Tools)
- OBS development headers / CMake config (`libobs`)

## Installing

### Option A: Build from source

```
build.bat
```

Double-click it, or run it from a shell. It requests admin elevation (via UAC), then: installs Visual Studio Build Tools if missing, downloads OBS development headers and generates an import library from your installed `obs.dll` if needed, configures and builds with CMake, and installs the result straight into your OBS installation. Restart OBS afterwards.

Manual build:
```bat
cmake -B build -G "Visual Studio 17 2022" -A x64 ^
      "-DOBS_INCLUDE_DIR=<path-to-obs-sdk>\libobs" ^
      "-DOBS_LIB=<path-to-obs-sdk>\obs.lib"
cmake --build build --config RelWithDebInfo
cmake --install build --config RelWithDebInfo --prefix "C:\Program Files\obs-studio"
```

### Option B: Manual install (pre-built binary)

| OS | Plugin binary | Data folder |
|---|---|---|
| **Windows** | `%ProgramFiles%\obs-studio\obs-plugins\64bit\obs-dynamic-autocrop.dll` | `%ProgramFiles%\obs-studio\data\obs-plugins\obs-dynamic-autocrop\` |

Then restart OBS.

## Adding the filter in OBS

1. Right-click your capture source in the Sources list -> **Filters**
2. The Filters window has two separate lists: **Audio/Video Filters** and **Effect Filters**. Click the **+** under **Effect Filters** specifically -- this is a render/video-processing filter, not an async source filter, so it won't appear under Audio/Video Filters.
3. Select **Dynamic Autocrop** from the list, then **Close**

## License

GPL-2.0 (see `LICENSE`). This links against `libobs`, which is GPL-2.0-or-later -- per OBS's own stated policy, any plugin that links against OBS Studio is required to be GPL-compatible.

## AI disclosure

This plugin's source code was written in collaboration with Claude (Anthropic). Every OBS API call it makes was checked against the actual OBS Studio source rather than assumed (see "Verification" above), and the shader/C-side interface was cross-checked symbol-by-symbol rather than shipped without checking it actually lines up.
