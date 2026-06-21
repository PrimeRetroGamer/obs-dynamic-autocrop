/*
 * dynamic_autocrop_filter.c
 *
 * OBS Studio plugin: Dynamic Autocrop, standalone. Intelligently crops the
 * black (or near-black) border produced by retro upscalers / line-doublers
 * (RetroTink, OSSC, Morph, and similar devices) running in a fixed-canvas
 * "scale under" style mode, then stretches the active picture to fill a
 * user-configured output resolution. See README.md for full documentation
 * of every setting.
 *
 * Detection pipeline, in order:
 *  - Brightness / Max Black % gates -- skip analysis on frames too dark or
 *    too uniformly black to give a trustworthy reading (loading screens,
 *    boot logos, fades).
 *  - Per-row/column content scan -- a row or column only counts as
 *    "content" once a meaningful fraction of its pixels exceed Black
 *    Threshold, not just a single stray pixel (resistant to analog noise
 *    spikes / hot pixels).
 *  - Max Crop X/Y -- applied FIRST, directly to the raw detected
 *    boundary, as a hard ceiling on how much of the frame can be cropped
 *    away per axis regardless of what detection finds, protecting
 *    genuine in-game black bars (cutscenes, title screens) from being
 *    mistaken for scaler border.
 *  - Horizontal/Vertical Trim % -- applied LAST, with nothing after it --
 *    independent extra inward trim per axis beyond the detected border
 *    (Genesis-style garbage pixels just outside the active picture, etc),
 *    always achieving its full configured effect regardless of Max Crop.
 *    Defaults to a small non-zero value rather than 0%, which also
 *    absorbs residual sub-analysis-cell boundary imprecision by default
 *    -- there's no separate hidden margin mechanism behind this one.
 *
 * Commit pipeline: every successful detection becomes a "pending
 * candidate" that needs Debounce consecutive matching passes before being
 * committed as the active crop (filtering out a single bad/transitional
 * frame, or a one-off noisy reading on a borderline edge). Two things
 * bypass debounce outright and commit a single reading unconditionally:
 *  - The very first detection ever (after creation or a resolution
 *    change) -- there's no prior crop to protect by waiting.
 *  - A stall-timeout safety valve (COMMIT_STALL_TIMEOUT_SECS) if real
 *    seconds since the last commit exceed it despite repeated valid
 *    attempts -- a true last resort, not the common path.
 *
 * Everything else -- a settings change, the Recalculate Crop Now button,
 * or the first reading after a dark/black-gated gap -- still goes through
 * genuine debounce confirmation, just FAST: fast_recheck re-triggers
 * analysis every FAST_RECHECK_INTERVAL_SECS instead of waiting for the
 * normal Recalc Interval timer, so `debounce_needed` consecutive
 * agreeing samples are gathered within a fraction of a second rather
 * than several.
 *
 * Output stage: the committed crop is applied via a GPU shader (Linear /
 * Point / Sharp Bilinear / Lanczos, user-selectable), inset by half a
 * texel on every side so the sampler can never bleed across the crop
 * boundary into the border row/column just outside it.
 *
 * Build: see CMakeLists.txt / README.md
 */

#include <obs-module.h>
#include <util/platform.h>
#include <graphics/graphics.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

OBS_DECLARE_MODULE()
OBS_MODULE_USE_DEFAULT_LOCALE("obs-dynamic-autocrop", "en-US")

MODULE_EXPORT const char *obs_module_description(void)
{
    return "Dynamic auto-crop for retro upscaler / capture card footage (RetroTink and similar devices)";
}

/* =============================================================================
 * Settings keys
 * ============================================================================= */
#define S_OUT_W         "output_width"
#define S_OUT_H         "output_height"
#define S_SCALE_FILTER  "scale_filter"
#define S_TRIM_X        "trim_x_pct"
#define S_TRIM_Y        "trim_y_pct"
#define S_BLACK_THRESH  "black_threshold"
#define S_RECALC_SECS   "recalc_interval"
#define S_MIN_BRIGHT    "min_brightness"
#define S_MAX_BLACK_PCT "max_black_percent"
#define S_DEBOUNCE      "debounce_count"
#define S_FREEZE_CROP   "freeze_crop"
#define S_RECALC_BUTTON "recalc_now_button"
#define S_LIMIT_SMALL_CHANGES "limit_small_changes"
#define S_MIN_CHANGE_PCT      "min_change_pct"
#define S_MAX_CROP_X    "max_crop_x"
#define S_MAX_CROP_Y    "max_crop_y"

/* Scale filter modes -- must match the dropdown order in filter_properties()
 * and the branch order in the pixel shader. */
#define SCALE_MODE_LINEAR         0
#define SCALE_MODE_POINT          1
#define SCALE_MODE_SHARP_BILINEAR 2
#define SCALE_MODE_LANCZOS        3

/* =============================================================================
 * Analysis downscale resolution
 *
 * The source frame is rendered down to this size before being read back to
 * the CPU for border detection. Larger = more precise edge detection
 * (especially for thin borders) at the cost of a bigger GPU readback.
 *
 * Deliberately 16:9 rather than 4:3 -- this plugin's actual sources
 * (RetroTink, OSSC, and similar scalers/capture devices) are essentially
 * always delivered on a 16:9 canvas, even when the active picture inside
 * it is 4:3. Matching the analysis grid's aspect ratio to that means both
 * axes get the same per-cell native-pixel coverage; a 4:3 analysis grid
 * against a 16:9 source spends its pixel budget unevenly, leaving
 * horizontal precision worse than vertical for no real reason. 1280x720
 * gets ~1.5 native pixels per analysis cell at 1080p on both axes (versus
 * 4x3 at the smaller grid this replaced) -- a real precision improvement
 * over a much larger jump (full native res) would cost. At higher source
 * resolutions (1440p, 4K) this also scales down proportionally, so it's
 * not 1080p-specific. Since this only runs periodically, not every frame,
 * the larger GPU readback (~3.5 MB versus well under 1 MB before) is a
 * reasonable trade for the precision gained.
 * ============================================================================= */
#define ANA_W 1280
#define ANA_H 720

/* If the committed crop hasn't updated in this many real seconds despite
 * repeated valid analysis attempts, the next attempt commits immediately
 * regardless of debounce matching. See secs_since_commit for why this
 * exists -- normal debounce has no time-based escape, so without this a
 * source with sustained drift or consistently-above-tolerance noise
 * between passes could leave the crop stuck on a stale value
 * indefinitely. Deliberately a flat constant rather than scaled from
 * Recalc Interval / Debounce -- "no update in 15 real seconds" is
 * unreasonable regardless of how those are configured. */
#define COMMIT_STALL_TIMEOUT_SECS 15.0f

/* Fixed per-pixel "is this black" cutoff used ONLY by the Max Black %
 * frame-level gate -- deliberately independent of the user's Black
 * Threshold setting (which tunes the border row/column scan instead).
 * Keeping these separate means raising Black Threshold for border
 * tuning never affects how readily Max Black % gates a frame. 16/255 is
 * a conservative "genuinely black" default that won't misclassify
 * normal dim/mid-tone gameplay. */
#define BLACK_PCT_GATE_THRESH 16

/* How long a new live resolution (when Output Width/Height = 0, auto) must
 * hold steady before being adopted as the reported/rendered output size.
 * Long enough to filter out brief capture-device negotiation blips on
 * startup or a mode change; short enough to feel immediate in practice. */
#define OUTPUT_SIZE_SETTLE_SECONDS 1.0f

/* Minimum real seconds between analysis passes while fast_recheck is
 * active. Without a cap, a source whose detected boundary doesn't settle
 * within 2 consecutive readings (not unusual -- that's the whole reason
 * debounce exists) would trigger a full GPU analysis pass (texrender +
 * a GPU->CPU readback + a 480x360 pixel scan) on every rendered frame,
 * for up to the full stall-timeout window, every time a setting changes
 * -- enough sustained load to make the rest of OBS feel sluggish. 100ms
 * keeps fast_recheck's responsiveness (imperceptible to a human
 * adjusting a slider) without hammering at full render framerate. */
#define FAST_RECHECK_INTERVAL_SECS 0.1f

/* =============================================================================
 * Crop / scale shader
 *
 * Remaps the output UV so that (0,0)-(1,1) covers only the detected crop
 * window inside the source, then samples it using one of three techniques
 * selected by `scale_mode`:
 *
 *   0 - Linear          : standard bilinear stretch (smooth, default)
 *   1 - Point            : nearest-neighbour (crisp, blocky pixels)
 *   2 - Sharp Bilinear   : nearest-neighbour pixel grid with a thin
 *                          anti-aliased blend only at texel edges -- the
 *                          common "sharp-bilinear" technique used by
 *                          retro-gaming upscalers/emulators.
 * ============================================================================= */
static const char *EFFECT_SRC =
    "uniform float4x4 ViewProj;\n"
    "uniform texture2d image;\n"
    "uniform float2 uv_offset;\n"
    "uniform float2 uv_scale;\n"
    "uniform float2 crop_px_size;\n"
    "uniform float2 output_px_size;\n"
    "uniform int    scale_mode;\n"
    "\n"
    "sampler_state linear_clamp {\n"
    "    Filter   = Linear;\n"
    "    AddressU = Clamp;\n"
    "    AddressV = Clamp;\n"
    "};\n"
    "\n"
    "sampler_state point_clamp {\n"
    "    Filter   = Point;\n"
    "    AddressU = Clamp;\n"
    "    AddressV = Clamp;\n"
    "};\n"
    "\n"
    "struct VertData {\n"
    "    float4 pos : POSITION;\n"
    "    float2 uv  : TEXCOORD0;\n"
    "};\n"
    "\n"
    "VertData VSMain(VertData v_in)\n"
    "{\n"
    "    VertData o;\n"
    "    o.pos = mul(float4(v_in.pos.xyz, 1.0), ViewProj);\n"
    "    o.uv  = v_in.uv;\n"   /* 0..1 over the crop region (output rect) */
    "    return o;\n"
    "}\n"
    "\n"
    "/* Sample at a CROP-LOCAL pixel coordinate (can be fractional, and can\n"
    " * fall slightly outside [0, crop_px_size] for filter taps near an\n"
    " * edge). Clamped to a half-texel-safe range so it can never reach\n"
    " * across the crop boundary into the border row/column outside it. */\n"
    "float4 sample_crop_px(float2 px)\n"
    "{\n"
    "    float2 half_texel = 0.5 / crop_px_size;\n"
    "    float2 cuv = clamp(px / crop_px_size, half_texel, 1.0 - half_texel);\n"
    "    float2 fuv = uv_offset + cuv * uv_scale;\n"
    "    return image.Sample(linear_clamp, fuv);\n"
    "}\n"
    "\n"
    "/* Lanczos-3 (6-tap) windowed-sinc weight, adapted from OBS's own\n"
    " * lanczos_scale.effect (itself adapted from the bsnes shader). */\n"
    "float lanczos_weight(float x)\n"
    "{\n"
    "    float x_pi = x * 3.141592654;\n"
    "    return 3.0 * sin(x_pi) * sin(x_pi * (1.0 / 3.0)) / (x_pi * x_pi);\n"
    "}\n"
    "\n"
    "void lanczos_weight6(float f_neg, out float3 tap012, out float3 tap345)\n"
    "{\n"
    "    tap012 = float3(\n"
    "        lanczos_weight(f_neg - 2.0),\n"
    "        lanczos_weight(f_neg - 1.0),\n"
    "        min(1.0, lanczos_weight(f_neg))); /* replace NaN-at-zero with 1.0 */\n"
    "    tap345 = float3(\n"
    "        lanczos_weight(f_neg + 1.0),\n"
    "        lanczos_weight(f_neg + 2.0),\n"
    "        lanczos_weight(f_neg + 3.0));\n"
    "\n"
    "    float sum = tap012.x + tap012.y + tap012.z + tap345.x + tap345.y + tap345.z;\n"
    "    float sum_i = 1.0 / sum;\n"
    "    tap012 = tap012 * sum_i;\n"
    "    tap345 = tap345 * sum_i;\n"
    "}\n"
    "\n"
    "/* Full 6x6-tap Lanczos resample, in crop-local pixel space. */\n"
    "float4 lanczos_sample(float2 crop_uv)\n"
    "{\n"
    "    float2 pos   = crop_uv * crop_px_size;\n"
    "    float2 pos2  = floor(pos - 0.5) + 0.5;\n"
    "    float2 f_neg = pos2 - pos;\n"
    "\n"
    "    float3 rowtap012, rowtap345;\n"
    "    lanczos_weight6(f_neg.x, rowtap012, rowtap345);\n"
    "    float3 coltap012, coltap345;\n"
    "    lanczos_weight6(f_neg.y, coltap012, coltap345);\n"
    "\n"
    "    float xpos0 = pos2.x - 2.0;\n"
    "    float xpos1 = pos2.x - 1.0;\n"
    "    float xpos2 = pos2.x;\n"
    "    float xpos3 = pos2.x + 1.0;\n"
    "    float xpos4 = pos2.x + 2.0;\n"
    "    float xpos5 = pos2.x + 3.0;\n"
    "\n"
    "    float ypos0 = pos2.y - 2.0;\n"
    "    float ypos1 = pos2.y - 1.0;\n"
    "    float ypos2 = pos2.y;\n"
    "    float ypos3 = pos2.y + 1.0;\n"
    "    float ypos4 = pos2.y + 2.0;\n"
    "    float ypos5 = pos2.y + 3.0;\n"
    "\n"
    "    float4 total = float4(0.0, 0.0, 0.0, 0.0);\n"
    "    float4 row;\n"
    "\n"
    "    row  = sample_crop_px(float2(xpos0, ypos0)) * rowtap012.x;\n"
    "    row += sample_crop_px(float2(xpos1, ypos0)) * rowtap012.y;\n"
    "    row += sample_crop_px(float2(xpos2, ypos0)) * rowtap012.z;\n"
    "    row += sample_crop_px(float2(xpos3, ypos0)) * rowtap345.x;\n"
    "    row += sample_crop_px(float2(xpos4, ypos0)) * rowtap345.y;\n"
    "    row += sample_crop_px(float2(xpos5, ypos0)) * rowtap345.z;\n"
    "    total += row * coltap012.x;\n"
    "\n"
    "    row  = sample_crop_px(float2(xpos0, ypos1)) * rowtap012.x;\n"
    "    row += sample_crop_px(float2(xpos1, ypos1)) * rowtap012.y;\n"
    "    row += sample_crop_px(float2(xpos2, ypos1)) * rowtap012.z;\n"
    "    row += sample_crop_px(float2(xpos3, ypos1)) * rowtap345.x;\n"
    "    row += sample_crop_px(float2(xpos4, ypos1)) * rowtap345.y;\n"
    "    row += sample_crop_px(float2(xpos5, ypos1)) * rowtap345.z;\n"
    "    total += row * coltap012.y;\n"
    "\n"
    "    row  = sample_crop_px(float2(xpos0, ypos2)) * rowtap012.x;\n"
    "    row += sample_crop_px(float2(xpos1, ypos2)) * rowtap012.y;\n"
    "    row += sample_crop_px(float2(xpos2, ypos2)) * rowtap012.z;\n"
    "    row += sample_crop_px(float2(xpos3, ypos2)) * rowtap345.x;\n"
    "    row += sample_crop_px(float2(xpos4, ypos2)) * rowtap345.y;\n"
    "    row += sample_crop_px(float2(xpos5, ypos2)) * rowtap345.z;\n"
    "    total += row * coltap012.z;\n"
    "\n"
    "    row  = sample_crop_px(float2(xpos0, ypos3)) * rowtap012.x;\n"
    "    row += sample_crop_px(float2(xpos1, ypos3)) * rowtap012.y;\n"
    "    row += sample_crop_px(float2(xpos2, ypos3)) * rowtap012.z;\n"
    "    row += sample_crop_px(float2(xpos3, ypos3)) * rowtap345.x;\n"
    "    row += sample_crop_px(float2(xpos4, ypos3)) * rowtap345.y;\n"
    "    row += sample_crop_px(float2(xpos5, ypos3)) * rowtap345.z;\n"
    "    total += row * coltap345.x;\n"
    "\n"
    "    row  = sample_crop_px(float2(xpos0, ypos4)) * rowtap012.x;\n"
    "    row += sample_crop_px(float2(xpos1, ypos4)) * rowtap012.y;\n"
    "    row += sample_crop_px(float2(xpos2, ypos4)) * rowtap012.z;\n"
    "    row += sample_crop_px(float2(xpos3, ypos4)) * rowtap345.x;\n"
    "    row += sample_crop_px(float2(xpos4, ypos4)) * rowtap345.y;\n"
    "    row += sample_crop_px(float2(xpos5, ypos4)) * rowtap345.z;\n"
    "    total += row * coltap345.y;\n"
    "\n"
    "    row  = sample_crop_px(float2(xpos0, ypos5)) * rowtap012.x;\n"
    "    row += sample_crop_px(float2(xpos1, ypos5)) * rowtap012.y;\n"
    "    row += sample_crop_px(float2(xpos2, ypos5)) * rowtap012.z;\n"
    "    row += sample_crop_px(float2(xpos3, ypos5)) * rowtap345.x;\n"
    "    row += sample_crop_px(float2(xpos4, ypos5)) * rowtap345.y;\n"
    "    row += sample_crop_px(float2(xpos5, ypos5)) * rowtap345.z;\n"
    "    total += row * coltap345.z;\n"
    "\n"
    "    return total;\n"
    "}\n"
    "\n"
    "float4 PSMain(VertData v_in) : TARGET\n"
    "{\n"
    "    float2 crop_uv = v_in.uv;\n"
    "    float2 half_texel = 0.5 / crop_px_size;\n"
    "    float2 final_crop_uv;\n"
    "    float2 final_uv;\n"
    "\n"
    "    if (scale_mode == 1) {\n"
    "        /* Point / Nearest -- snap to the centre of the source texel,\n"
    "         * clamped to a valid texel index so the very last output\n"
    "         * pixel can never round up past the crop boundary. */\n"
    "        float2 texel_idx   = clamp(floor(crop_uv * crop_px_size),\n"
    "                                    float2(0.0, 0.0),\n"
    "                                    crop_px_size - float2(1.0, 1.0));\n"
    "        float2 texel_coord = texel_idx + 0.5;\n"
    "        final_crop_uv = texel_coord / crop_px_size;\n"
    "        final_uv = uv_offset + final_crop_uv * uv_scale;\n"
    "        return image.Sample(point_clamp, final_uv);\n"
    "    } else if (scale_mode == 2) {\n"
    "        /* Sharp Bilinear -- nearest-neighbour grid with a narrow\n"
    "         * anti-aliased blend only right at texel boundaries. The\n"
    "         * final clamp keeps linear filtering from ever reaching\n"
    "         * across the crop edge into the border row outside it. */\n"
    "        float2 tex_coord_px = crop_uv * crop_px_size;\n"
    "        float2 scale        = output_px_size / crop_px_size;\n"
    "        float2 scale_clamped = max(scale, float2(1.0, 1.0));\n"
    "        float2 freq          = frac(tex_coord_px);\n"
    "        float2 sharpened     = saturate((freq - 0.5) * scale_clamped + 0.5);\n"
    "        float2 final_px      = floor(tex_coord_px) + sharpened;\n"
    "        final_crop_uv = clamp(final_px / crop_px_size,\n"
    "                               half_texel, 1.0 - half_texel);\n"
    "        final_uv = uv_offset + final_crop_uv * uv_scale;\n"
    "        return image.Sample(linear_clamp, final_uv);\n"
    "    } else if (scale_mode == 3) {\n"
    "        /* Lanczos -- highest quality, most expensive (36 texture\n"
    "         * samples per output pixel, every frame). */\n"
    "        return lanczos_sample(crop_uv);\n"
    "    }\n"
    "\n"
    "    /* Linear (default) -- smooth bilinear stretch, inset by half a\n"
    "     * source texel on every side. Without this inset, the GPU's\n"
    "     * linear filter can blend 50/50 with the texel just OUTSIDE the\n"
    "     * crop region at the exact boundary (Clamp addressing only\n"
    "     * stops sampling outside the full texture, not outside our\n"
    "     * crop sub-rectangle) -- which is what produced the thin\n"
    "     * leftover line of border colour regardless of Horizontal/Vertical Trim. */\n"
    "    final_crop_uv = clamp(crop_uv, half_texel, 1.0 - half_texel);\n"
    "    final_uv = uv_offset + final_crop_uv * uv_scale;\n"
    "    return image.Sample(linear_clamp, final_uv);\n"
    "}\n"
    "\n"
    "technique Draw\n"
    "{\n"
    "    pass\n"
    "    {\n"
    "        vertex_shader = VSMain(v_in);\n"
    "        pixel_shader  = PSMain(v_in);\n"
    "    }\n"
    "}\n";

/* =============================================================================
 * Filter instance data
 * ============================================================================= */
struct dynamic_autocrop_filter {
    obs_source_t *context;

    /* -- User settings -- */
    uint32_t out_w_setting;   /* 0 = auto (match OBS canvas resolution) */
    uint32_t out_h_setting;   /* 0 = auto (match OBS canvas resolution) */
    int      scale_mode;      /* SCALE_MODE_*                          */
    float    trim_x;          /* Horizontal Trim %, as a fraction (0.01=1%) */
    float    trim_y;          /* Vertical Trim %, as a fraction (0.01=1%)   */
    int      black_thresh;    /* max(r,g,b) < thresh -> treated as black */
    float    recalc_secs;     /* seconds between analyses              */
    float    min_brightness;  /* 0-1 avg-luma gate; skip darker frames */
    float    max_black_pct;   /* 0-1 fraction of frame; skip if more of
                                * the frame than this is near-black, even
                                * if average luma alone wouldn't catch it
                                * (e.g. dark stealth scenes with a small
                                * bright HUD pulling the average up)     */
    int      debounce_needed; /* stable consecutive samples to commit  */
    bool     freeze_crop;     /* true = stop periodic re-analysis, hold
                                * whatever crop is currently committed   */
    float    max_crop_x;      /* 0-1: max fraction of WIDTH allowed to be
                                * cropped away, protects genuine in-game
                                * black bars (title screens, cutscenes)
                                * from being mistaken for scaler border  */
    float    max_crop_y;      /* same, for HEIGHT                       */

    /* -- Ignore Small Changes --
     * When enabled, a freshly debounce-confirmed RAW candidate is
     * compared against the CURRENTLY COMMITTED raw boundary too (not
     * just against itself across consecutive readings, which is what
     * debounce alone already does) -- if the difference is below
     * min_change_pct on every edge, the candidate is discarded and the
     * existing committed crop is kept exactly as-is. Debounce protects
     * against trusting a single noisy reading; it does NOT protect
     * against the noise floor itself drifting -- two consecutive
     * readings can perfectly agree with each other at a position
     * that's still only noise relative to what's already committed,
     * and debounce alone has no way to tell the difference. This adds
     * that second comparison specifically. */
    bool     limit_small_changes;
    float    min_change_pct;  /* 0-1 fraction; differences below this on
                                * every edge are treated as noise        */

    /* -- Committed RAW detected boundary (normalised UV 0-1) --
     * Deliberately RAW -- i.e. BEFORE Horizontal/Vertical Trim
     * or Max Crop X/Y. Those are applied fresh on every render frame by
     * apply_crop_postprocessing() instead, as a pure function of these
     * values plus whatever the CURRENT settings are, so changing them
     * never depends on a new detection happening to succeed. Debounce
     * (below) operates on these raw values too -- confirming the genuine
     * detected signal is stable is the part that actually needs multiple
     * agreeing samples; the post-processing math applied on top of it is
     * deterministic and doesn't need separate confirmation. */
    float cx0, cy0;   /* top-left  */
    float cx1, cy1;   /* bot-right */

    /* -- Pending RAW candidate (debounce buffer) -- */
    float px0, py0;
    float px1, py1;
    int   stable_count;

    /* -- Has a crop ever been committed for the current resolution?
     * The very first successful detection commits immediately
     * (bypassing debounce) so the picture isn't stuck showing the
     * uncropped border for several analysis cycles after startup or
     * a resolution change. Subsequent updates still use debounce to
     * avoid jitter. -- */
    bool committed_once;

    /* -- Fast-recheck mode --
     * Set whenever the user explicitly changes a setting, presses
     * Recalculate Crop Now, or the first valid reading arrives after
     * one or more dark/black-gated skips. While true, filter_tick
     * re-triggers analysis every FAST_RECHECK_INTERVAL_SECS instead of
     * waiting for the normal Recalc Interval timer, so debounce's
     * usual multi-sample confirmation resolves within a fraction of a
     * second rather than multiple full Recalc Interval cycles --
     * without skipping debounce itself, which still genuinely requires
     * `debounce_needed` consecutive matching reads, just gathered
     * faster. */
    bool  fast_recheck;
    float fast_recheck_elapsed; /* seconds since the last fast-recheck pass */

    /* -- Stall-prevention safety valve --
     * Normal debounce compares each new analysis result only against
     * the PREVIOUS pending candidate, never against elapsed time. If
     * the source has any sustained drift (not just random jitter) --
     * or noise that happens to consistently exceed CROP_TOLERANCE
     * between consecutive passes -- stable_count can keep resetting to
     * 1 forever, with nothing ever confirming. There is no theoretical
     * upper bound on how long the committed crop can then sit stale
     * without normal debounce alone. This tracks real seconds since
     * the last successful commit; if it grows past
     * COMMIT_STALL_TIMEOUT_SECS despite repeated valid (non-skipped)
     * analysis attempts, the next analysis commits unconditionally,
     * regardless of whether it matches the pending candidate. This
     * remains a true last-resort bypass (unlike fast_recheck above) --
     * it only fires if even rapid-fire debounce couldn't agree for an
     * unreasonably long stretch. */
    float secs_since_commit;

    /* -- Tracked source dimensions (detect resolution changes) -- */
    uint32_t prev_src_w;
    uint32_t prev_src_h;

    /* -- Debounced stable output size (runtime only, never saved) --
     * Used only when Output Width/Height = 0 (auto). A new live
     * resolution must hold steady for OUTPUT_SIZE_SETTLE_SECONDS before
     * being adopted -- this filters out brief negotiation blips during
     * capture device startup/mode changes without permanently locking
     * onto a possibly-wrong value the way a one-time snapshot would. */
    uint32_t stable_out_w, stable_out_h; /* currently adopted stable size */
    uint32_t candidate_w, candidate_h;   /* most recent live reading      */
    float    settle_timer;               /* seconds candidate has held    */

    /* -- Timing / state flags -- */
    float elapsed;
    bool  needs_analysis;
    bool  first_run;

    /* -- GPU resources -- */
    gs_texrender_t *texrender;  /* downscale render target (ANA_W x ANA_H) */
    gs_stagesurf_t *stagesurf;  /* CPU-readable copy                       */

    /* -- Crop/scale effect -- */
    gs_effect_t *effect;
    gs_eparam_t *ep_uv_offset;
    gs_eparam_t *ep_uv_scale;
    gs_eparam_t *ep_crop_px_size;
    gs_eparam_t *ep_output_px_size;
    gs_eparam_t *ep_scale_mode;
};

/* =============================================================================
 * Utility
 * ============================================================================= */
static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* =============================================================================
 * Pixel analysis
 *
 * Reads the staged ANA_W x ANA_H BGRA buffer and locates the content
 * boundary by scanning from each edge inward.
 *
 * Returns false when:
 *   - The average luma is below min_brightness (dark / fade-to-black).
 *   - The detected content region is implausibly small (< 10 % per axis).
 * ============================================================================= */
static bool analyse_frame(struct dynamic_autocrop_filter *f,
                           uint32_t src_w, uint32_t src_h,
                           float *out_x0, float *out_y0,
                           float *out_x1, float *out_y1)
{
    uint8_t  *data     = NULL;
    uint32_t  linesize = 0;

    if (!gs_stagesurface_map(f->stagesurf, &data, &linesize))
        return false;

    const uint32_t W = ANA_W;
    const uint32_t H = ANA_H;
    const int      T = f->black_thresh;

    /* -- Brightness gate + black-pixel-coverage gate --
     * OBS stage surfaces in BGRA layout:
     *   byte 0 = B, 1 = G, 2 = R, 3 = A
     * BT.601 luma = 0.299 R + 0.587 G + 0.114 B
     *
     * Both metrics are computed in a single pass:
     *   - avg_luma:   mean brightness across the whole frame. Catches
     *                 uniformly dark frames (fades, loading screens).
     *   - black_frac: fraction of PIXELS that are individually below
     *                 BLACK_PCT_GATE_THRESH (a fixed constant, NOT the
     *                 user's Black Threshold -- see below for why).
     *                 Catches scenes that are mostly black but contain
     *                 a small bright element (HUD, muzzle flash, a
     *                 flashlight cone) that pulls the average luma up
     *                 enough to slip past that gate alone.
     *
     * black_frac deliberately does NOT reuse the user's Black Threshold
     * (T) the way the border row/column scan does. Those are two
     * different concepts that happen to look similar: Black Threshold
     * tunes "how dark must a pixel be to count as scaler BORDER", while
     * this gate asks "is the FRAME overall too dark to trust an
     * analysis on at all". Sharing one value meant raising Black
     * Threshold for legitimate border-detection tuning could also
     * silently make this gate misclassify genuine mid-tone gameplay as
     * "black", pushing black_frac over Max Black % and freezing
     * analysis entirely -- with no way to tell from the Black Threshold
     * tooltip alone that the two were connected. */
    double  luma_sum         = 0.0;
    uint64_t black_pixel_count = 0;
    for (uint32_t y = 0; y < H; y++) {
        const uint8_t *row = data + y * linesize;
        for (uint32_t x = 0; x < W; x++) {
            uint8_t b = row[x * 4 + 0];
            uint8_t g = row[x * 4 + 1];
            uint8_t r = row[x * 4 + 2];

            luma_sum += 0.114 * b + 0.587 * g + 0.299 * r;

            uint8_t maxc = b;
            if (g > maxc) maxc = g;
            if (r > maxc) maxc = r;
            if (maxc <= (uint8_t)BLACK_PCT_GATE_THRESH)
                black_pixel_count++;
        }
    }
    double avg_luma   = luma_sum / ((double)W * H * 255.0);
    double black_frac = (double)black_pixel_count / ((double)W * H);

    if (avg_luma < (double)f->min_brightness) {
        gs_stagesurface_unmap(f->stagesurf);
        blog(LOG_INFO,
             "[dynamic-autocrop] Frame too dark (luma=%.3f < %.3f), skipping",
             avg_luma, (double)f->min_brightness);
        return false;
    }

    if (black_frac > (double)f->max_black_pct) {
        gs_stagesurface_unmap(f->stagesurf);
        blog(LOG_INFO,
             "[dynamic-autocrop] Frame too black (%.1f%% black > %.1f%% limit), skipping",
             black_frac * 100.0, (double)f->max_black_pct * 100.0);
        return false;
    }

    /* -- Content / border pixel test --
     * A pixel is "content" if any of its R, G, B channels exceeds the
     * black threshold. This naturally handles:
     *   - Pure-black borders
     *   - Slightly grey or noisy analog-capture borders
     *   - The composite/S-Video noise floor
     */
#define IS_CONTENT(row, x) \
    ((row)[(x) * 4 + 0] > (uint8_t)T || \
     (row)[(x) * 4 + 1] > (uint8_t)T || \
     (row)[(x) * 4 + 2] > (uint8_t)T)

    /* A row/column only counts as "content" once at least this fraction of
     * its pixels exceed the black threshold (floor of 2 pixels minimum).
     * Without this, a SINGLE noisy/hot pixel anywhere in a border row --
     * an analog noise spike, a compression artifact, a stray bright dot --
     * is enough to make the whole row register as content and stop the
     * scan immediately, leaving every genuinely-black row above/below it
     * still inside the committed crop. Requiring a minimum spread of
     * bright pixels means real picture content (which lights up many
     * pixels across the row) still triggers normally, while isolated
     * noise in the border does not. */
#define MIN_CONTENT_FRACTION 0.02f

    uint32_t min_count_w = (uint32_t)((float)W * MIN_CONTENT_FRACTION);
    if (min_count_w < 2) min_count_w = 2;
    uint32_t min_count_h = (uint32_t)((float)H * MIN_CONTENT_FRACTION);
    if (min_count_h < 2) min_count_h = 2;

    /* Top edge */
    uint32_t top = 0;
    for (uint32_t y = 0; y < H; y++) {
        const uint8_t *row = data + y * linesize;
        uint32_t count = 0;
        for (uint32_t x = 0; x < W; x++)
            if (IS_CONTENT(row, x)) count++;
        if (count >= min_count_w) { top = y; break; }
    }

    /* Bottom edge */
    uint32_t bot = H > 0 ? H - 1 : 0;
    for (int32_t y = (int32_t)H - 1; y >= 0; y--) {
        const uint8_t *row = data + y * linesize;
        uint32_t count = 0;
        for (uint32_t x = 0; x < W; x++)
            if (IS_CONTENT(row, x)) count++;
        if (count >= min_count_w) { bot = (uint32_t)y; break; }
    }

    /* Left edge */
    uint32_t lft = 0;
    for (uint32_t x = 0; x < W; x++) {
        uint32_t count = 0;
        for (uint32_t y = 0; y < H; y++) {
            const uint8_t *row = data + y * linesize;
            if (IS_CONTENT(row, x)) count++;
        }
        if (count >= min_count_h) { lft = x; break; }
    }

    /* Right edge */
    uint32_t rgt = W > 0 ? W - 1 : 0;
    for (int32_t x = (int32_t)W - 1; x >= 0; x--) {
        uint32_t count = 0;
        for (uint32_t y = 0; y < H; y++) {
            const uint8_t *row = data + y * linesize;
            if (IS_CONTENT(row, x)) count++;
        }
        if (count >= min_count_h) { rgt = (uint32_t)x; break; }
    }

#undef IS_CONTENT
#undef MIN_CONTENT_FRACTION

    gs_stagesurface_unmap(f->stagesurf);

    /* Convert pixel coords -> normalised UV. These are the RAW detected
     * boundary -- Horizontal/Vertical Trim and Max Crop X/Y are
     * deliberately NOT applied here. See apply_crop_postprocessing() and
     * its call site in filter_render for why: those are pure functions
     * of the CURRENT settings and should always reflect the latest
     * values instantly, with no dependency on whether a fresh detection
     * happened to run this frame. Baking them in here would tie them to
     * this function's own gating (Min Brightness / Max Black %) and
     * rate-limiting (Recalc Interval / fast_recheck) for no reason --
     * changing Horizontal/Vertical Trim or Max Crop X/Y doesn't need a new
     * detection at all, just a recompute from whatever raw boundary was
     * last found, which is exactly what moving them out of here enables.
     */
    float x0 = (float)lft        / (float)W;
    float y0 = (float)top        / (float)H;
    float x1 = (float)(rgt + 1)  / (float)W;
    float y1 = (float)(bot + 1)  / (float)H;

    /* Sanity check: content must cover at least 2 % in each axis. This
     * only needs to catch truly degenerate results (a near-zero-size
     * detection from some analysis glitch) -- it is NOT meant to
     * second-guess a legitimately small-but-real detection. Keep this
     * threshold low: at higher Black Threshold values, genuine (if dim)
     * gameplay can start failing the per-pixel brightness test too, not
     * just the actual border, shrinking the detected region -- a higher
     * threshold here would reject those passes outright, silently
     * freezing the crop with no obvious cause. */
    if ((x1 - x0) < 0.02f || (y1 - y0) < 0.02f) {
        blog(LOG_WARNING,
             "[dynamic-autocrop] Detected region too small (%.3f x %.3f), skipping -- "
             "if this repeats, Black Threshold may be set too high for this source",
             x1 - x0, y1 - y0);
        return false;
    }

    blog(LOG_INFO,
         "[dynamic-autocrop] Raw border detected: (%.4f,%.4f)-(%.4f,%.4f)",
         x0, y0, x1, y1);

    *out_x0 = x0;
    *out_y0 = y0;
    *out_x1 = x1;
    *out_y1 = y1;
    return true;
}

/* =============================================================================
 * Crop post-processing -- Max Crop X/Y, Horizontal/Vertical Trim
 * =============================================================================
 * Pure function: takes a raw detected boundary plus the filter's CURRENT
 * settings and produces the final crop to actually render. Deliberately
 * has no dependency on whether a fresh detection happened recently, no
 * gating, no rate-limiting, and no side effects (besides the optional
 * diagnostic log) -- it's meant to be cheap enough to call on every
 * single rendered frame, so that Horizontal/Vertical Trim and Max Crop X/Y always
 * reflect the latest value instantly, the moment they're changed,
 * completely independent of the (rate-limited, gated) detection pipeline
 * that produces the raw boundary it works from. This is what Max Crop
 * X/Y already needed fixed once before (a settings change had no visible
 * effect until detection happened to succeed again, which Min Brightness
 * / Max Black % could block indefinitely) -- Horizontal/Vertical Trim had exactly
 * the same latent bug, just less obviously, since changing it usually
 * does still trigger a fast-tracked fresh detection most of the time,
 * masking the underlying issue except when gating got in the way.
 * ============================================================================= */
static void apply_crop_postprocessing(struct dynamic_autocrop_filter *f,
                                       float raw_x0, float raw_y0,
                                       float raw_x1, float raw_y1,
                                       uint32_t src_w, uint32_t src_h,
                                       bool do_log,
                                       float *out_x0, float *out_y0,
                                       float *out_x1, float *out_y1)
{
    float x0 = raw_x0, y0 = raw_y0, x1 = raw_x1, y1 = raw_y1;

    /* -- Max Crop X/Y, applied FIRST, directly to the RAW detected
     * boundary -- and Horizontal/Vertical Trim, applied LAST, with
     * nothing after it --
     *
     * They serve different, unrelated purposes: Max Crop protects
     * against bad DETECTION (e.g. misreading a legitimate letterboxed
     * title screen as scaler border), while Horizontal/Vertical Trim is
     * an intentional, user-configured additional trim layered on top,
     * for things detection inherently can't catch (Genesis-style
     * garbage pixels just past the active picture). Applying Max Crop
     * only to the raw boundary, with Trim layered on afterward and
     * nothing checking it again, keeps them fully independent: Max Crop
     * always protects the raw detection, and Trim always achieves its
     * full configured effect, regardless of whatever Max Crop X/Y is
     * set to. */
    float min_width  = 1.0f - f->max_crop_x;
    float min_height = 1.0f - f->max_crop_y;

    float raw_width  = x1 - x0;
    float raw_height = y1 - y0;
    bool clamped_by_max_crop = false;

    /* When the clamp engages, it expands outward from wherever the
     * detected boundary already is, rather than forcing the result to
     * the exact frame middle (0.5). The split between the two sides is
     * therefore dynamic, not a fixed 50/50 -- if the active picture is
     * genuinely positioned a bit higher or lower (or left/right) within
     * the frame, the extra room "given back" by the clamp comes out
     * asymmetric to match, instead of cutting evenly into both sides
     * regardless of where the real content sits.
     *
     * This is deliberately based on raw_x0..raw_y1 -- the boundary
     * passed in here has already cleared debounce (see do_analysis)
     * before ever reaching this function, meaning it already agreed
     * with itself across several consecutive readings before being
     * trusted at all. Any asymmetry still present at this point isn't
     * single-frame noise; it's a stable, repeated signal, almost always
     * reflecting a real property of the source (genuine overscan/border
     * asymmetry is common) rather than something to discard. */
    if (raw_width < min_width) {
        float deficit = (min_width - raw_width) * 0.5f;
        x0 = raw_x0 - deficit;
        x1 = raw_x1 + deficit;
        clamped_by_max_crop = true;
    }
    if (raw_height < min_height) {
        float deficit = (min_height - raw_height) * 0.5f;
        y0 = raw_y0 - deficit;
        y1 = raw_y1 + deficit;
        clamped_by_max_crop = true;
    }

    float post_maxcrop_x0 = x0, post_maxcrop_y0 = y0, post_maxcrop_x1 = x1, post_maxcrop_y1 = y1; /* for diagnostics */

    /* -- Horizontal / Vertical Trim --
     * Shrinks the (Max-Crop-protected) crop window inward, independently
     * per axis. Handles consoles that place coloured garbage pixels
     * (Mega Drive, etc.) just outside the active picture area, lets the
     * user compensate for any letterboxing/pillarboxing asymmetry
     * directly, and -- since it defaults to a non-zero value rather than
     * 0% -- absorbs residual sub-analysis-cell boundary imprecision by
     * default too, the same role a separate hidden floor used to serve.
     * There's deliberately no other margin mechanism behind this one:
     * what's configured here is the whole story, with nothing extra
     * applied invisibly underneath it. Each axis is just its own
     * percentage of its own RAW detected dimension -- nothing shared
     * between axes, and nothing for Max Crop X/Y to influence, since
     * this is computed from raw_x0..raw_y1 (the original detected
     * boundary) rather than from x0..y1 as they stand here (which may
     * have already been pulled outward by Max Crop above). */
    float expand_x = (raw_x1 - raw_x0) * f->trim_x;
    float expand_y = (raw_y1 - raw_y0) * f->trim_y;

    x0 += expand_x;
    x1 -= expand_x;
    y0 += expand_y;
    y1 -= expand_y;

    /* Nothing after this point checks or limits the result -- Horizontal/Vertical
     * Trim is the final word, always. */

    if (do_log) {
        /* Rate-limited by the caller (only logged once per fresh raw
         * detection, not every render frame) -- full before/after
         * breakdown of every stage, so a reported "trim doesn't seem to
         * apply" can be checked directly against real numbers instead
         * of guessed at. */
        blog(LOG_INFO,
             "[dynamic-autocrop] postprocess: raw=(%.4f,%.4f)-(%.4f,%.4f) max-crop-clamped=%s "
             "-> post-maxcrop=(%.4f,%.4f)-(%.4f,%.4f) trim_x=%.2f%% trim_y=%.2f%% "
             "-> final=(%.4f,%.4f)-(%.4f,%.4f) [max_crop_x=%.2f%% max_crop_y=%.2f%%]",
             raw_x0, raw_y0, raw_x1, raw_y1,
             clamped_by_max_crop ? "YES" : "no",
             post_maxcrop_x0, post_maxcrop_y0, post_maxcrop_x1, post_maxcrop_y1,
             f->trim_x * 100.0f, f->trim_y * 100.0f,
             clampf(x0, 0.f, 1.f), clampf(y0, 0.f, 1.f), clampf(x1, 0.f, 1.f), clampf(y1, 0.f, 1.f),
             f->max_crop_x * 100.0f, f->max_crop_y * 100.0f);
    }

    *out_x0 = clampf(x0, 0.f, 1.f);
    *out_y0 = clampf(y0, 0.f, 1.f);
    *out_x1 = clampf(x1, 0.f, 1.f);
    *out_y1 = clampf(y1, 0.f, 1.f);
}

/* =============================================================================
 * Debounce helpers
 * ============================================================================= */
#define CROP_TOLERANCE 0.015f  /* 1.5 % per axis is "close enough" */

static inline bool crops_match(float ax0, float ay0, float ax1, float ay1,
                                float bx0, float by0, float bx1, float by1,
                                float tolerance)
{
    return fabsf(ax0 - bx0) < tolerance &&
           fabsf(ay0 - by0) < tolerance &&
           fabsf(ax1 - bx1) < tolerance &&
           fabsf(ay1 - by1) < tolerance;
}

/* =============================================================================
 * GPU analysis pass
 *
 * Called from filter_render when needs_analysis is true. Renders the
 * source into a small texture, copies to a stage surface, reads back pixels
 * on the CPU, runs the analysis, then updates the debounce state.
 *
 * Must be called from the graphics/render thread (no obs_enter_graphics).
 * ============================================================================= */
static void do_analysis(struct dynamic_autocrop_filter *f,
                         obs_source_t *target,
                         uint32_t src_w, uint32_t src_h)
{
    if (!src_w || !src_h)
        return;

    /* Recreate stage surface if it's missing */
    if (!f->stagesurf) {
        f->stagesurf = gs_stagesurface_create(ANA_W, ANA_H, GS_BGRA);
        if (!f->stagesurf) {
            blog(LOG_ERROR, "[dynamic-autocrop] Failed to create stage surface");
            return;
        }
    }

    /* Recreate texrender if missing */
    if (!f->texrender) {
        f->texrender = gs_texrender_create(GS_BGRA, GS_ZS_NONE);
        if (!f->texrender) {
            blog(LOG_ERROR, "[dynamic-autocrop] Failed to create texrender");
            return;
        }
    }

    /* -- Render source -> small texture --
     * We render the filter's target (the source / filters below us in the
     * chain) into a small BGRA texrender. The ortho projection maps the
     * source's native resolution into the texrender's pixel space so the
     * hardware automatically downscales with bilinear filtering.
     */
    gs_texrender_reset(f->texrender);
    if (!gs_texrender_begin(f->texrender, ANA_W, ANA_H))
        return;

    struct vec4 clear_col;
    vec4_zero(&clear_col);
    gs_clear(GS_CLEAR_COLOR, &clear_col, 0.f, 0);
    gs_ortho(0.f, (float)src_w, 0.f, (float)src_h, -100.f, 100.f);

    obs_source_video_render(target);  /* renders filter chain below us */

    gs_texrender_end(f->texrender);

    /* -- Stage (GPU -> CPU) -- */
    gs_stage_texture(f->stagesurf, gs_texrender_get_texture(f->texrender));

    /* -- Analyse -- */
    float nx0, ny0, nx1, ny1;
    if (!analyse_frame(f, src_w, src_h, &nx0, &ny0, &nx1, &ny1)) {
        /* Frame too dark or degenerate -- keep current crop, but
         * fast-track debounce on the next valid reading: there's no
         * recent valid baseline left to meaningfully compare against
         * after an indeterminate gap, and the scene may have genuinely
         * changed during it (a loading screen ending into real
         * gameplay, for instance). */
        f->fast_recheck         = true;
        f->fast_recheck_elapsed = 0.f;
        return;
    }

    /* Diagnostic only -- shows what the CURRENT settings would produce
     * for this freshly detected raw boundary. Naturally rate-limited by
     * how often raw detection itself happens (not called every render
     * frame), unlike the actual rendered crop, which recomputes this
     * silently on every frame via apply_crop_postprocessing() in
     * filter_render so Horizontal/Vertical Trim / Max Crop X/Y always reflect the
     * latest setting instantly regardless of detection timing. */
    {
        float dbg_x0, dbg_y0, dbg_x1, dbg_y1;
        apply_crop_postprocessing(f, nx0, ny0, nx1, ny1, src_w, src_h, true,
                                   &dbg_x0, &dbg_y0, &dbg_x1, &dbg_y1);
    }

    /* -- Commit logic --
     * Only two things ever bypass debounce outright and commit a
     * single reading unconditionally:
     *
     *   1. The very first successful detection ever (after creation or
     *      a resolution change) -- there's no prior crop to protect by
     *      waiting, so showing SOMETHING immediately beats sitting on
     *      the full uncropped frame for several cycles.
     *   2. The stall-timeout safety valve, as an absolute last resort
     *      (see secs_since_commit) -- only fires after real debounce
     *      (even fast-tracked) has failed to agree for an unreasonably
     *      long stretch.
     *
     * Everything else -- including settings changes, the Recalculate
     * button, and the first reading after a dark/black gap -- goes
     * through the SAME genuine debounce confirmation below as normal
     * periodic analysis, just gathered faster via fast_recheck (see its
     * own comment) so it resolves within a fraction of a second rather
     * than a full Recalc Interval, without skipping the requirement for
     * genuine agreement between independent readings.
     */
    bool stall_timeout = f->secs_since_commit >= COMMIT_STALL_TIMEOUT_SECS;

    if (!f->committed_once || stall_timeout) {
        f->cx0 = nx0; f->cy0 = ny0;
        f->cx1 = nx1; f->cy1 = ny1;
        f->px0 = nx0; f->py0 = ny0;
        f->px1 = nx1; f->py1 = ny1;
        f->stable_count      = 0;
        f->committed_once    = true;
        f->fast_recheck       = false;
        f->secs_since_commit = 0.f;

        if (stall_timeout) {
            blog(LOG_WARNING,
                 "[dynamic-autocrop] Crop forced through after %.1fs without an update (stall timeout) "
                 "L=%.4f T=%.4f R=%.4f B=%.4f",
                 COMMIT_STALL_TIMEOUT_SECS, f->cx0, f->cy0, f->cx1, f->cy1);
        } else {
            blog(LOG_INFO,
                 "[dynamic-autocrop] Initial crop committed  L=%.4f T=%.4f R=%.4f B=%.4f",
                 f->cx0, f->cy0, f->cx1, f->cy1);
        }
        return;
    }

    /* -- Debounce (genuine, every time past the bootstrap case) --
     * Only commit a new crop after it has been stable for
     * `debounce_needed` consecutive analysis passes. This prevents a
     * single transitional frame (cut, wipe, flash, or a noisy one-off
     * reading) from snapping the crop to a bad value. fast_recheck
     * (set in filter_tick / filter_update) just controls how quickly
     * those consecutive passes are gathered -- it never weakens the
     * agreement requirement itself.
     */
    if (crops_match(nx0, ny0, nx1, ny1,
                    f->px0, f->py0, f->px1, f->py1, CROP_TOLERANCE)) {
        f->stable_count++;
        /* Still track the LATEST reading even on a match, not just the
         * first-in-streak value. Without this, while a setting is being
         * actively adjusted (e.g. dragging the Horizontal/Vertical Trim slider --
         * each frame's reading differs slightly from the last but still
         * falls within tolerance of the streak's starting point), the
         * eventual commit applies a stale value from whenever the
         * streak began rather than wherever the slider actually ended
         * up, making the crop visibly lag behind the live setting. */
        f->px0 = nx0; f->py0 = ny0;
        f->px1 = nx1; f->py1 = ny1;
    } else {
        /* New candidate -- restart countdown */
        f->px0 = nx0; f->py0 = ny0;
        f->px1 = nx1; f->py1 = ny1;
        f->stable_count = 1;
    }

    blog(LOG_INFO,
         "[dynamic-autocrop] debounce: candidate=(%.4f,%.4f)-(%.4f,%.4f) stable_count=%d/%d fast_recheck=%s",
         f->px0, f->py0, f->px1, f->py1, f->stable_count, f->debounce_needed,
         f->fast_recheck ? "on" : "off");

    if (f->stable_count >= f->debounce_needed) {
        /* -- Ignore Small Changes --
         * Debounce above already confirmed this candidate is stable
         * (matches itself across consecutive readings) -- but that says
         * nothing about whether it's actually DIFFERENT from what's
         * already committed. Two consecutive noisy readings can easily
         * agree with each other at a position that's still only noise
         * relative to the current crop. This is a second, independent
         * comparison: against the CURRENTLY COMMITTED boundary, not
         * against itself. */
        if (f->limit_small_changes &&
            crops_match(f->px0, f->py0, f->px1, f->py1,
                        f->cx0, f->cy0, f->cx1, f->cy1, f->min_change_pct)) {
            /* Below threshold on every edge -- treat as noise, keep the
             * committed crop exactly as it is. Still resolve this
             * cycle's bookkeeping normally (reset stable_count,
             * fast_recheck, secs_since_commit) so repeatedly-ignored
             * noise-sized changes don't make the stall-timeout safety
             * valve think detection is genuinely stuck and force one
             * through regardless. */
            f->stable_count      = 0;
            f->fast_recheck       = false;
            f->secs_since_commit = 0.f;

            blog(LOG_INFO,
                 "[dynamic-autocrop] Confirmed change too small (< %.2f%%), ignoring -- "
                 "kept L=%.4f T=%.4f R=%.4f B=%.4f",
                 f->min_change_pct * 100.0f, f->cx0, f->cy0, f->cx1, f->cy1);
            return;
        }

        f->cx0 = f->px0; f->cy0 = f->py0;
        f->cx1 = f->px1; f->cy1 = f->py1;
        f->stable_count      = 0;
        f->fast_recheck       = false; /* confirmed -- back to normal cadence */
        f->secs_since_commit = 0.f;

        blog(LOG_INFO,
             "[dynamic-autocrop] Crop updated  L=%.4f T=%.4f R=%.4f B=%.4f",
             f->cx0, f->cy0, f->cx1, f->cy1);
    } else {
        /* Didn't commit this pass -- secs_since_commit accrues real
         * elapsed time in filter_tick instead of being approximated
         * here, since during fast_recheck this can be called many
         * times per second rather than once per Recalc Interval. */
    }
}

/* =============================================================================
 * OBS source callbacks
 * ============================================================================= */

static const char *filter_get_name(void *unused)
{
    UNUSED_PARAMETER(unused);
    return "Dynamic Autocrop";
}

/* -- Defaults -- */
static void filter_get_defaults(obs_data_t *settings)
{
    obs_data_set_default_int   (settings, S_OUT_W,         0);    /* 0 = auto, match canvas */
    obs_data_set_default_int   (settings, S_OUT_H,         0);    /* 0 = auto, match canvas */
    obs_data_set_default_int   (settings, S_SCALE_FILTER,  SCALE_MODE_LANCZOS);
    obs_data_set_default_int   (settings, S_BLACK_THRESH,  50);   /* ~20 % luma */
    obs_data_set_default_double(settings, S_TRIM_X,        0.5);  /* 0.5 %   */
    obs_data_set_default_double(settings, S_TRIM_Y,        0.5);  /* 0.5 %   */
    obs_data_set_default_double(settings, S_RECALC_SECS,   1.0);  /* 1 s     */
    obs_data_set_default_double(settings, S_MIN_BRIGHT,    15.0); /* 15 %    */
    obs_data_set_default_double(settings, S_MAX_BLACK_PCT, 60.0); /* 60 %    */
    obs_data_set_default_int   (settings, S_DEBOUNCE,      4);    /* 4 passes */
    obs_data_set_default_bool  (settings, S_FREEZE_CROP,   false);
    obs_data_set_default_double(settings, S_MAX_CROP_X,    60.0); /* 60 %    */
    obs_data_set_default_double(settings, S_MAX_CROP_Y,    20.0); /* 20 %    */
    obs_data_set_default_bool  (settings, S_LIMIT_SMALL_CHANGES, true);
    obs_data_set_default_double(settings, S_MIN_CHANGE_PCT,      3.0);  /* 3 %     */
}

/* -- Update (settings changed) -- */
static void filter_update(void *data, obs_data_t *settings)
{
    struct dynamic_autocrop_filter *f = data;

    f->out_w_setting  = (uint32_t)obs_data_get_int(settings, S_OUT_W);
    f->out_h_setting  = (uint32_t)obs_data_get_int(settings, S_OUT_H);
    f->scale_mode     = (int)     obs_data_get_int(settings, S_SCALE_FILTER);
    f->black_thresh   = (int)     obs_data_get_int(settings, S_BLACK_THRESH);
    f->trim_x         = (float)obs_data_get_double(settings, S_TRIM_X) / 100.f;
    f->trim_y         = (float)obs_data_get_double(settings, S_TRIM_Y) / 100.f;
    f->recalc_secs    = (float)obs_data_get_double(settings, S_RECALC_SECS);
    f->min_brightness = (float)obs_data_get_double(settings, S_MIN_BRIGHT)   / 100.f;
    f->max_black_pct  = (float)obs_data_get_double(settings, S_MAX_BLACK_PCT)/ 100.f;
    f->debounce_needed= (int)     obs_data_get_int(settings, S_DEBOUNCE);
    f->freeze_crop    = obs_data_get_bool(settings, S_FREEZE_CROP);
    f->max_crop_x     = (float)obs_data_get_double(settings, S_MAX_CROP_X) / 100.f;
    f->max_crop_y     = (float)obs_data_get_double(settings, S_MAX_CROP_Y) / 100.f;
    f->limit_small_changes = obs_data_get_bool(settings, S_LIMIT_SMALL_CHANGES);
    f->min_change_pct      = (float)obs_data_get_double(settings, S_MIN_CHANGE_PCT) / 100.f;

    /* Clamp recalc to a sane minimum to avoid GPU hammering */
    if (f->recalc_secs < 0.5f)
        f->recalc_secs = 0.5f;

    /* Trigger a fresh, fast-tracked analysis so setting changes take
     * effect quickly -- but not while frozen, since freezing
     * specifically means "don't change the crop"; the Recalculate Crop
     * Now button is the explicit escape hatch for that. fast_recheck
     * makes confirmation happen within a fraction of a second instead
     * of a full Recalc Interval, without skipping debounce's agreement
     * requirement, so a single noisy reading still can't commit on its
     * own. */
    if (!f->freeze_crop) {
        f->needs_analysis       = true;
        f->fast_recheck         = true;
        f->fast_recheck_elapsed = 0.f;
    }
}

/* -- Recalculate Crop Now button (visible only while Freeze Crop is on) -- */
static bool filter_recalc_button_clicked(obs_properties_t *props, obs_property_t *property, void *data)
{
    UNUSED_PARAMETER(props);
    UNUSED_PARAMETER(property);
    struct dynamic_autocrop_filter *f = data;
    f->needs_analysis       = true;
    f->fast_recheck         = true;
    f->fast_recheck_elapsed = 0.f;
    return false; /* no property layout change needed */
}

/* -- Toggle Recalculate button visibility live as Freeze Crop is checked/unchecked -- */
static bool filter_freeze_modified(obs_properties_t *props, obs_property_t *property, obs_data_t *settings)
{
    UNUSED_PARAMETER(property);
    bool frozen = obs_data_get_bool(settings, S_FREEZE_CROP);
    obs_property_t *btn = obs_properties_get(props, S_RECALC_BUTTON);
    if (btn)
        obs_property_set_visible(btn, frozen);
    return true;
}

/* -- Toggle Min Change % slider visibility live as Ignore Small Changes is checked/unchecked -- */
static bool filter_limit_small_changes_modified(obs_properties_t *props, obs_property_t *property, obs_data_t *settings)
{
    UNUSED_PARAMETER(property);
    bool enabled = obs_data_get_bool(settings, S_LIMIT_SMALL_CHANGES);
    obs_property_t *slider = obs_properties_get(props, S_MIN_CHANGE_PCT);
    if (slider)
        obs_property_set_visible(slider, enabled);
    return true;
}

/* -- Create -- */
static void *filter_create(obs_data_t *settings, obs_source_t *source)
{
    struct dynamic_autocrop_filter *f = bzalloc(sizeof(*f));
    f->context = source;

    /* Sane "no-op" crop defaults */
    f->cx0 = 0.f; f->cy0 = 0.f;
    f->cx1 = 1.f; f->cy1 = 1.f;
    f->px0 = 0.f; f->py0 = 0.f;
    f->px1 = 1.f; f->py1 = 1.f;

    f->committed_once = false;
    f->first_run       = true;
    f->needs_analysis   = false;
    f->elapsed          = 0.f;
    f->stable_count      = 0;
    f->prev_src_w        = 0;
    f->prev_src_h        = 0;

    /* Debounced stable-output-size tracking starts empty; filter_tick
     * populates it from the live source over the next few ticks. See
     * the struct field comments and filter_tick for the settle-time
     * logic this uses. */
    f->stable_out_w = 0;
    f->stable_out_h = 0;
    f->candidate_w  = 0;
    f->candidate_h  = 0;
    f->settle_timer = 0.f;

    /* Create GPU resources inside graphics context */
    obs_enter_graphics();

    char *errors = NULL;
    f->effect = gs_effect_create(EFFECT_SRC, "dynamic_autocrop_effect.effect", &errors);
    if (!f->effect) {
        blog(LOG_ERROR, "[dynamic-autocrop] Shader compile error: %s",
             errors ? errors : "(unknown)");
        bfree(errors);
        obs_leave_graphics();
        bfree(f);
        return NULL;
    }
    bfree(errors);

    f->ep_uv_offset        = gs_effect_get_param_by_name(f->effect, "uv_offset");
    f->ep_uv_scale         = gs_effect_get_param_by_name(f->effect, "uv_scale");
    f->ep_crop_px_size     = gs_effect_get_param_by_name(f->effect, "crop_px_size");
    f->ep_output_px_size   = gs_effect_get_param_by_name(f->effect, "output_px_size");
    f->ep_scale_mode       = gs_effect_get_param_by_name(f->effect, "scale_mode");

    obs_leave_graphics();

    filter_update(f, settings);
    return f;
}

/* -- Destroy -- */
static void filter_destroy(void *data)
{
    struct dynamic_autocrop_filter *f = data;
    obs_enter_graphics();
    gs_effect_destroy(f->effect);
    gs_texrender_destroy(f->texrender);
    gs_stagesurface_destroy(f->stagesurf);
    obs_leave_graphics();
    bfree(f);
}

/* -- Tick (main thread, runs every frame) -- */
static void filter_tick(void *data, float t)
{
    struct dynamic_autocrop_filter *f = data;

    /* -- Debounced stable-resolution tracking for auto (0) output sizing --
     * Only matters when Output Width or Height is 0 (auto); runs every
     * tick regardless of first_run/freeze state, since it's about
     * output SIZE, not crop analysis.
     *
     * Targets the OBS CANVAS resolution (base_width/base_height), not
     * the source's own native resolution. Whenever canvas and source
     * resolutions differ, sizing to the source instead means OBS has to
     * apply its own additional resize afterward to fit the canvas -- a
     * second resampling pass happening entirely outside this filter,
     * after it's already handed off a finished image, which can
     * introduce a faint artifact at the crop edge and costs image
     * quality from stacking two resizes. Targeting the canvas
     * resolution directly means this filter does the full crop+resize
     * in one fully-controlled pass, and OBS has nothing left to resize.
     *
     * A new reading must hold steady for OUTPUT_SIZE_SETTLE_SECONDS
     * before being adopted, filtering out transient blips (a brief
     * negotiation hiccup, or someone changing OBS's canvas resolution
     * mid-session) without permanently locking onto a too-early
     * reading the way a one-time snapshot would. */
    if (f->out_w_setting == 0 || f->out_h_setting == 0) {
        uint32_t live_w = 0, live_h = 0;

        struct obs_video_info ovi;
        if (obs_get_video_info(&ovi)) {
            live_w = ovi.base_width;
            live_h = ovi.base_height;
        } else {
            /* Should be rare -- canvas info momentarily unavailable.
             * Fall back to the source's own resolution rather than
             * leaving live_w/live_h at 0, which would stall sizing
             * entirely until the next tick. */
            obs_source_t *size_target = obs_filter_get_target(f->context);
            if (size_target) {
                live_w = obs_source_get_base_width(size_target);
                live_h = obs_source_get_base_height(size_target);
            }
        }

        if (live_w && live_h) {
            if (live_w != f->candidate_w || live_h != f->candidate_h) {
                f->candidate_w  = live_w;
                f->candidate_h  = live_h;
                f->settle_timer = 0.f;
            } else {
                f->settle_timer += t;
            }

            bool no_stable_yet = (f->stable_out_w == 0 || f->stable_out_h == 0);
            if (no_stable_yet || f->settle_timer >= OUTPUT_SIZE_SETTLE_SECONDS) {
                f->stable_out_w = f->candidate_w;
                f->stable_out_h = f->candidate_h;
            }
        }
    }

    /* First frame after creation: analyse immediately. This happens even
     * if Freeze Crop is on, since freezing is about holding a crop once
     * we HAVE one -- a brand new filter still needs an initial detection
     * rather than sitting uncropped forever. */
    if (f->first_run) {
        f->first_run      = false;
        f->needs_analysis = true;
        return;
    }

    /* While frozen, the periodic timer is suppressed entirely -- the
     * committed crop just holds. The "Recalculate Crop Now" button
     * (visible only while Freeze Crop is checked) is the only thing
     * that can trigger another analysis pass. */
    if (f->freeze_crop)
        return;

    /* Real elapsed time since the last successful commit, for the
     * stall-timeout safety valve in do_analysis. Only accrues while
     * not frozen, since no analysis runs during a freeze -- nothing
     * can be "stalled" if nothing is being attempted. Reset to 0
     * wherever an actual commit happens. Tracked here (real per-tick
     * delta time) rather than approximated per analysis call, since
     * fast_recheck below can make do_analysis run many times per
     * second rather than once per Recalc Interval. */
    f->secs_since_commit += t;

    if (f->fast_recheck) {
        /* A settings change, the Recalculate button, or the first
         * reading after a dark/black gap is pending fast confirmation
         * -- re-trigger analysis at FAST_RECHECK_INTERVAL_SECS instead
         * of waiting for the normal timer, so debounce's multi-sample
         * agreement resolves within a fraction of a second rather than
         * multiple Recalc Interval cycles. Real debounce still applies
         * in do_analysis; this only controls how fast samples are
         * gathered, not whether they need to agree. Rate-limited rather
         * than running every rendered frame to avoid sustained GPU/CPU
         * load on sources whose boundary takes a while to settle (see
         * FAST_RECHECK_INTERVAL_SECS). */
        f->fast_recheck_elapsed += t;
        if (f->fast_recheck_elapsed >= FAST_RECHECK_INTERVAL_SECS) {
            f->fast_recheck_elapsed = 0.f;
            f->needs_analysis       = true;
        }
        return;
    }

    f->elapsed += t;
    if (f->elapsed >= f->recalc_secs) {
        f->elapsed        = 0.f;
        f->needs_analysis = true;
    }
}

/* -- Width / height --
 * Reports the OUTPUT size of this filter to OBS. If the user has set an
 * explicit output resolution, that size is reported (and used for the
 * stretch in filter_render). Otherwise we report the debounced stable
 * resolution tracked in filter_tick (the OBS canvas resolution by
 * default -- see the tracking block there for why) -- critically, the
 * SAME value filter_render uses for the actual draw, so reported size
 * and rendered size can never mismatch (a mismatch is what produces a
 * correctly proportioned image rendered small inside a larger black
 * canvas).
 */
static uint32_t filter_get_width(void *data)
{
    struct dynamic_autocrop_filter *f = data;
    if (f->out_w_setting != 0)
        return f->out_w_setting;
    return f->stable_out_w;
}

static uint32_t filter_get_height(void *data)
{
    struct dynamic_autocrop_filter *f = data;
    if (f->out_h_setting != 0)
        return f->out_h_setting;
    return f->stable_out_h;
}

/* -- Render -- */
static void filter_render(void *data, gs_effect_t *effect)
{
    UNUSED_PARAMETER(effect);
    struct dynamic_autocrop_filter *f = data;

    obs_source_t *target = obs_filter_get_target(f->context);
    if (!target) {
        obs_source_skip_video_filter(f->context);
        return;
    }

    uint32_t src_w = obs_source_get_base_width(target);
    uint32_t src_h = obs_source_get_base_height(target);
    if (!src_w || !src_h) {
        obs_source_skip_video_filter(f->context);
        return;
    }

    /* If the source resolution changed, reset to full-frame crop and
     * force a fresh analysis on the new resolution. */
    if (src_w != f->prev_src_w || src_h != f->prev_src_h) {
        f->cx0 = 0.f; f->cy0 = 0.f;
        f->cx1 = 1.f; f->cy1 = 1.f;
        f->stable_count    = 0;
        f->committed_once  = false;
        f->needs_analysis  = true;
        f->prev_src_w      = src_w;
        f->prev_src_h      = src_h;
    }

    /* -- Analysis pass (rate-limited) -- */
    if (f->needs_analysis) {
        f->needs_analysis = false;
        do_analysis(f, target, src_w, src_h);
    }

    /* Resolve the actual output size: user setting, or the debounced
     * stable resolution tracked in filter_tick (the same value
     * filter_get_width/height report -- keeping these in lockstep is
     * what guarantees the reported size always matches what's actually
     * drawn). Falls back to the immediate live src_w/h only in the rare
     * case this runs before filter_tick has established a stable value
     * yet, so the very first frame never renders at 0x0. */
    uint32_t out_w = (f->out_w_setting != 0) ? f->out_w_setting : (f->stable_out_w ? f->stable_out_w : src_w);
    uint32_t out_h = (f->out_h_setting != 0) ? f->out_h_setting : (f->stable_out_h ? f->stable_out_h : src_h);

    /* -- Output render --
     * obs_source_process_filter_begin renders all filters below us in the
     * chain into a texture and returns true. We then set our shader
     * params and hand the texture + effect to process_filter_end, which
     * draws a sprite sized out_w x out_h using our shader -- this is what
     * performs the "stretch to fill" when out_w/out_h differ from the
     * cropped region's own aspect ratio.
     */
    if (!obs_source_process_filter_begin(f->context, GS_RGBA,
                                          OBS_NO_DIRECT_RENDERING)) {
        obs_source_skip_video_filter(f->context);
        return;
    }

    /* -- Crop post-processing, re-applied here independently of analysis --
     * f->cx0..f->cy1 hold the RAW debounce-confirmed detected boundary
     * only -- Horizontal/Vertical Trim and Max Crop X/Y are applied
     * fresh, every single render frame, directly against whatever the
     * CURRENT settings are. This is what guarantees Horizontal/Vertical
     * Trim and Max Crop X/Y always take effect the instant they're
     * changed, with zero dependency on whether a new raw detection has
     * happened to run -- which Min Brightness / Max Black % gating, or
     * simply Recalc Interval's pacing, could otherwise delay
     * indefinitely. f->cx0..f->cy1 themselves stay untouched here, since
     * debounce in do_analysis needs that genuine raw detection history
     * to compare fresh readings against. */
    float rx0, ry0, rx1, ry1;
    apply_crop_postprocessing(f, f->cx0, f->cy0, f->cx1, f->cy1, src_w, src_h, false,
                               &rx0, &ry0, &rx1, &ry1);

    struct vec2 uv_off = { rx0,       ry0       };
    struct vec2 uv_scl = { rx1 - rx0, ry1 - ry0 };
    struct vec2 crop_px = {
        (rx1 - rx0) * (float)src_w,
        (ry1 - ry0) * (float)src_h
    };
    struct vec2 out_px = { (float)out_w, (float)out_h };

    gs_effect_set_vec2(f->ep_uv_offset,      &uv_off);
    gs_effect_set_vec2(f->ep_uv_scale,       &uv_scl);
    gs_effect_set_vec2(f->ep_crop_px_size,   &crop_px);
    gs_effect_set_vec2(f->ep_output_px_size, &out_px);
    gs_effect_set_int (f->ep_scale_mode,     f->scale_mode);

    obs_source_process_filter_end(f->context, f->effect, out_w, out_h);
}

/* =============================================================================
 * Properties UI
 * ============================================================================= */
static obs_properties_t *filter_properties(void *data)
{
    struct dynamic_autocrop_filter *f = data;
    obs_properties_t *props = obs_properties_create();

    obs_property_t *p;

    p = obs_properties_add_int(props, S_OUT_W, "Output Width", 0, 7680, 1);
    obs_property_set_long_description(p, "0 = match your OBS canvas resolution automatically (recommended).");

    p = obs_properties_add_int(props, S_OUT_H, "Output Height", 0, 4320, 1);
    obs_property_set_long_description(p, "0 = match your OBS canvas resolution automatically (recommended).");

    obs_property_t *sf = obs_properties_add_list(props, S_SCALE_FILTER,
        "Scale Filter",
        OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);
    obs_property_list_add_int(sf, "Linear (Smooth)",            SCALE_MODE_LINEAR);
    obs_property_list_add_int(sf, "Point (Nearest / Crisp Pixels)", SCALE_MODE_POINT);
    obs_property_list_add_int(sf, "Sharp Bilinear (Retro)",     SCALE_MODE_SHARP_BILINEAR);
    obs_property_list_add_int(sf, "Lanczos (Highest Quality, Heaviest)", SCALE_MODE_LANCZOS);

    p = obs_properties_add_int_slider(props, S_BLACK_THRESH, "Black Threshold", 0, 64, 1);
    obs_property_set_long_description(p, "How dark a pixel must be to count as border. Raise for noisy/composite captures.\nOnly affects border detection -- the Max Black % gate below uses its own fixed definition, independent of this.");

    p = obs_properties_add_float_slider(props, S_TRIM_X, "Horizontal Trim %", 0.0, 10.0, 0.1);
    obs_property_set_long_description(p, "Extra inward trim from the left and right edges, beyond the detected border.\nUse for stray garbage pixels just outside the picture (e.g. Mega Drive/Genesis dots).\nDefaults to a small non-zero value to prevent a thin border sliver -- lowering it toward 0% trades that protection away.");

    p = obs_properties_add_float_slider(props, S_TRIM_Y, "Vertical Trim %", 0.0, 10.0, 0.1);
    obs_property_set_long_description(p, "Extra inward trim from the top and bottom edges, beyond the detected border.\nUse for stray garbage pixels just outside the picture.\nDefaults to a small non-zero value to prevent a thin border sliver -- lowering it toward 0% trades that protection away.");

    p = obs_properties_add_float_slider(props, S_RECALC_SECS, "Recalc Interval (s)", 0.5, 30.0, 0.5);
    obs_property_set_long_description(p, "How often (seconds) to re-scan for the border.");

    p = obs_properties_add_float_slider(props, S_MIN_BRIGHT, "Min Brightness %", 0.0, 50.0, 0.5);
    obs_property_set_long_description(p, "Skip re-scanning frames darker than this\n(average brightness across the whole frame).");

    p = obs_properties_add_float_slider(props, S_MAX_BLACK_PCT, "Max Black %", 0.0, 100.0, 1.0);
    obs_property_set_long_description(p, "Skip re-scanning if more of the frame than this is near-black.\nCatches dark scenes with a bright HUD that Min Brightness alone would miss.");

    p = obs_properties_add_int_slider(props, S_DEBOUNCE, "Debounce", 1, 6, 1);
    obs_property_set_long_description(p, "Stable samples required before applying a CHANGED crop.");

    p = obs_properties_add_float_slider(props, S_MAX_CROP_X, "Max Crop X %", 0.0, 90.0, 1.0);
    obs_property_set_long_description(p, "Never crop away more than this much of the frame's WIDTH, even if detection finds more.\nProtects genuine in-game black bars (e.g. TATE-mode vertical shooters like Ikaruga, played pillarboxed) from being over-cropped.");

    p = obs_properties_add_float_slider(props, S_MAX_CROP_Y, "Max Crop Y %", 0.0, 90.0, 1.0);
    obs_property_set_long_description(p, "Never crop away more than this much of the frame's HEIGHT, even if detection finds more.\nProtects genuine in-game black bars (e.g. title screens like Metroid Prime) from being over-cropped.");

    p = obs_properties_add_bool(props, S_LIMIT_SMALL_CHANGES, "Ignore Small Changes");
    obs_property_set_long_description(p, "Skip applying a new crop if it's only marginally different from the current one -- helps avoid tiny adjustments caused by analog noise.");
    obs_property_set_modified_callback(p, filter_limit_small_changes_modified);

    p = obs_properties_add_float_slider(props, S_MIN_CHANGE_PCT, "Min Change %", 0.1, 5.0, 0.1);
    obs_property_set_long_description(p, "How different (per edge) a new crop must be from the current one before it's treated as a real change rather than noise.");
    obs_property_set_visible(p, f ? f->limit_small_changes : false);

    p = obs_properties_add_bool(props, S_FREEZE_CROP, "Freeze Crop");
    obs_property_set_long_description(p, "Stop re-scanning and hold the current crop exactly as-is.\nUseful as a workaround for games that otherwise confuse detection.");
    obs_property_set_modified_callback(p, filter_freeze_modified);

    obs_property_t *btn = obs_properties_add_button(props, S_RECALC_BUTTON,
        "Recalculate Crop Now", filter_recalc_button_clicked);
    obs_property_set_visible(btn, f ? f->freeze_crop : false);

    return props;
}

/* =============================================================================
 * Plugin registration
 * ============================================================================= */
static struct obs_source_info dynamic_autocrop_filter_info = {
    .id             = "dynamic_autocrop_filter",
    .type           = OBS_SOURCE_TYPE_FILTER,
    .output_flags   = OBS_SOURCE_VIDEO,
    .get_name       = filter_get_name,
    .create         = filter_create,
    .destroy        = filter_destroy,
    .update         = filter_update,
    .video_tick     = filter_tick,
    .video_render   = filter_render,
    .get_width      = filter_get_width,
    .get_height     = filter_get_height,
    .get_properties = filter_properties,
    .get_defaults   = filter_get_defaults,
};

bool obs_module_load(void)
{
    obs_register_source(&dynamic_autocrop_filter_info);
    blog(LOG_INFO, "[dynamic-autocrop] Plugin v1.0.0 loaded (Dynamic Autocrop, standalone)");
    return true;
}

void obs_module_unload(void)
{
    blog(LOG_INFO, "[dynamic-autocrop] Plugin unloaded");
}
