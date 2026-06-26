# Dynamic Autocrop

An OBS Studio plugin that automatically detects and crops the black border produced by retro upscalers (RetroTink, OSSC, Morph, and similar devices) running in a fixed-canvas "scale under" mode, then stretches the active picture to fill your configured output resolution in real time.

**Tested on:** PlayStation 2 (v1.1), GameCube (v1.0) via RetroTink 5X Pro.

## The problem it fixes

Retro upscalers and line-doublers output a fixed canvas resolution — e.g. 1920×1080 — with the actual game picture sitting inside it, surrounded by a black border. This happens any time the source resolution doesn't fill the output canvas: RetroTink's "Scale: Under" mode, OSSC's line-doubling output at lower resolutions, or any manual crop setup where the picture doesn't fill the frame. The border size varies by console, by game, and by scaler settings. OBS's built-in Crop/Pad filter crops a fixed pixel count and can't adapt when it changes. This plugin detects the real border each scan and crops to it dynamically.

## How it works

### Detection

Every scan interval, the filter renders the source into a small 640×360 staging buffer and reads it back to the CPU:

1. **Per-edge zone gating** — each edge (top, bottom, left, right) has its own sampling zone, set by Edge Sampling Region X/Y %. The average luma of non-pure-black pixels in that zone must meet or exceed Max Darkness % for analysis to proceed. All four zones must pass simultaneously — if any zone is gated, the scan is held and the current crop stays. This prevents loading screens, fades, and dark scenes from triggering a false crop update.

2. **Per-row/column content scan** — scans inward from each edge. A row or column counts as content once a meaningful fraction of its pixels exceed Black Threshold, so a single noise spike or hot pixel can't trigger a false border position on its own.

Horizontal/Vertical Trim and Max Crop X/Y are applied at render time only, completely independently of detection. They never affect what the scanner sees or what debounce compares.

### Commit pipeline

A freshly detected crop goes through debounce before applying: it must match the previous scan within 1.5% on all four edges for `Debounce` consecutive passes. This stops a single bad or transitional frame from snapping the picture to a wrong crop.

Two things bypass debounce:
- **Force Crop Now button** — bypasses both the darkness gate and debounce, committing whatever is detected on the next pass directly. Useful when detection is gated on a dark border you know is stable.
- **Default Crop dropdown** — when you change the Default Crop setting, that position commits immediately as the active crop. Detection then runs normally from there, tightening inward via debounce as usual.

When gates are open and a new crop is being confirmed, the scan interval drops to 100ms (fast recheck) until debounce settles, then returns to the configured Scan Interval.

If **Skip Minor Updates** is enabled, a confirmed crop is also compared against what's currently showing — if every edge differs by less than Minimum Update Size %, the update is skipped and the current crop holds.

### Post-processing

Applied fresh every rendered frame, independently of detection:

1. **Max Crop X/Y** — hard ceiling on how much of the frame can be removed per axis. Enforced first, expanding the crop back outward if detection would remove more than the limit.
2. **Horizontal/Vertical Trim** — inward trim applied last. Nothing overrides it afterward.

Both take effect instantly when you change them — even while Freeze Crop is on or the gate is holding.

### Output

The final crop is applied via a GPU shader using the selected Scale Filter, with a built-in sub-pixel safety margin so a thin sliver of border can never bleed into the picture edge.

## Per-console setup with Scene Collections

Filter settings are saved per scene collection by OBS automatically. The recommended workflow for multiple consoles:

1. Set up and tune the filter for your first console.
2. In OBS: **Scene Collection → Duplicate** and name it after the console.
3. Open the duplicate, retune the filter for the next console.
4. Switch collections when switching consoles — the filter loads with the correct settings automatically.

## Verification

Every OBS API used here was checked against the real OBS 31.0.0 source (`obsproject/obs-studio` tag `31.0.0`) rather than assumed — including a direct symbol-by-symbol cross-check between every shader uniform declared and every uniform looked up on the C side, confirming neither has an orphaned or missing entry. The plugin compiles cleanly against those real headers, and the shader source is verified brace/paren-balanced as part of the same check.

## Project layout

```
obs-dynamic-autocrop/
  CMakeLists.txt       -- standalone, out-of-tree build
  build.bat            -- elevates, then runs build.ps1 (recommended entry point)
  build.ps1            -- Windows build script (see below)
  src/
    dynamic_autocrop_filter.c -- the filter itself (detection, post-processing, shader)
  data/
    locale/en-US.ini    -- filter display name shown in OBS (required)
```

## Settings

| Setting | Default | What it does |
|---|---|---|
| **Output Width / Height** | 0 (auto) | Resolution the crop is stretched to fill. `0` matches your OBS canvas — recommended, since it lets the filter do crop and resize in one pass. |
| **Scale Filter** | Lanczos | Resampling quality: Point (nearest-neighbour), Linear, Sharp Bilinear, or Lanczos (highest quality). |
| **Default Crop** | None | Starting crop position used when detection resets (resolution change or filter added). 4:3 removes pillarbox on a 16:9 source; 16:9 removes letterbox on a 4:3 source. Detection can only tighten from this position, never widen past it. |
| **Black Threshold** | 45 | Per-pixel brightness cutoff. A pixel where max(R,G,B) ≤ this value is treated as black border; above it counts as content. Raise for noisy composite/S-Video captures where black isn't truly 0. |
| **Max Crop X %** | 60 | Hard ceiling on how much width detection can remove. Protects intentional pillarboxing (e.g. TATE-mode shooters). |
| **Max Crop Y %** | 30 | Hard ceiling on how much height detection can remove. Protects intentional letterboxing (e.g. cinematic title screens). |
| **Scan Interval (s)** | 1.0 | How often the filter re-scans for borders when idle. Drops to 100ms automatically while confirming a new crop. |
| **Max Darkness %** | 6 | Minimum average luma a zone must have (excluding pure-black pixels) to allow analysis. Raise to hold the crop during darker scenes. |
| **Edge Sampling Region Y %** | 12 | How far in from top and bottom the zone gate samples. Raise if your source has deep borders keeping those zones gated. |
| **Edge Sampling Region X %** | 20 | How far in from left and right the zone gate samples. Raise if your source has wide borders keeping those zones gated. |
| **Debounce** | 4 | Consecutive matching scans required before a changed crop applies. |
| **Skip Minor Updates** | On | Skip applying a new crop if every edge differs by less than Minimum Update Size % from the current crop. |
| **Minimum Update Size %** | 3 | Visible when Skip Minor Updates is on. Per-edge threshold below which a confirmed crop is considered noise and discarded. |
| **Freeze Crop** | Off | Hold the current crop exactly and stop scanning. Post-processing (Trim, Max Crop) still applies every frame. |
| **Force Crop Now** | — | Button. Bypasses the darkness gate and debounce, committing whatever is detected on the next pass immediately. |
| **Horizontal Trim %** | 0.4 | Extra inward trim from left and right edges after detection. Useful for garbage pixels at the border edge (e.g. Mega Drive border dots). |
| **Vertical Trim %** | 0.4 | Same as Horizontal Trim, for top and bottom. |

## Requirements

- **OBS Studio 28.0** or later
- **CMake 3.16+**
- MSVC 2019+ (Visual Studio Build Tools)
- OBS development headers (`libobs`)

## Building

```
build.bat
```

Double-click or run from a shell. Requests UAC elevation, then: installs Visual Studio Build Tools if missing, downloads OBS development headers, configures and builds with CMake, and installs directly into your OBS installation. Restart OBS afterward.

Manual build:
```bat
cmake -B build -G "Visual Studio 17 2022" -A x64 ^
      "-DOBS_INCLUDE_DIR=<path-to-obs-sdk>\libobs" ^
      "-DOBS_LIB=<path-to-obs-sdk>\obs.lib"
cmake --build build --config RelWithDebInfo
cmake --install build --config RelWithDebInfo --prefix "C:\Program Files\obs-studio"
```

## Installing

| OS | Plugin binary | Data folder |
|---|---|---|
| **Windows** | `%ProgramFiles%\obs-studio\obs-plugins\64bit\obs-dynamic-autocrop.dll` | `%ProgramFiles%\obs-studio\data\obs-plugins\obs-dynamic-autocrop\` |

Restart OBS after installing.

## Adding the filter

1. Right-click your capture source in the Sources list → **Filters**
2. The Filters window has two separate lists: **Audio/Video Filters** and **Effect Filters**. Click the **+** under **Effect Filters** specifically — this is a render/video-processing filter, not an async source filter, so it won't appear under Audio/Video Filters.
3. Select **Dynamic Autocrop** from the list and adjust settings as needed.

## License

GPL-2.0 (see `LICENSE`). This plugin links against `libobs`, which is GPL-2.0-or-later.

## AI disclosure

This plugin was written in collaboration with Claude (Anthropic). Every OBS API call was verified against the actual OBS Studio 31.0.0 source, and the shader/C interface was cross-checked symbol-by-symbol.
