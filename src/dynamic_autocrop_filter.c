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
 *  - Max Darkness % gate -- skip analysis on frames too dark or too
 *    uniformly black to give a trustworthy reading (loading screens,
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
 * frame, or a one-off noisy reading on a borderline edge). The first
 * detection ever (after creation or a resolution change) commits
 * immediately -- there is no prior crop to protect by waiting.
 *
 * Everything else -- a settings change, the Force Crop Now button,
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
#ifdef _WIN32
#  include <util/threading-windows.h>
#else
#  include <util/threading-posix.h>
#endif
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
#define S_MAX_DARKNESS  "max_darkness"
#define S_DEFAULT_CROP  "default_crop"
#define S_EDGE_SAMPLE_Y "edge_sample_y"
#define S_EDGE_SAMPLE_X "edge_sample_x"
#define S_DEBOUNCE      "debounce_count"
#define S_FREEZE_CROP   "freeze_crop"
#define S_RECALC_BUTTON "recalc_now_button"
#define S_LIMIT_SMALL_CHANGES "skip_minor_updates"
#define S_MIN_CHANGE_PCT      "min_update_size"
#define S_MAX_CROP_X    "max_crop_x"
#define S_MAX_CROP_Y    "max_crop_y"

/* Default crop modes */
#define DEFAULT_CROP_NONE  0   /* full frame, let detection decide */
#define DEFAULT_CROP_4_3   1   /* 4:3 center crop (removes pillarbox on 16:9) */
#define DEFAULT_CROP_16_9  2   /* 16:9 center crop (removes letterbox on 4:3) */

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
#define ANA_W 640
#define ANA_H 360

/* How long a new live resolution (when Output Width/Height = 0, auto) must
 * hold steady before being adopted as the reported/rendered output size.
 * Long enough to filter out brief capture-device negotiation blips on
 * startup or a mode change; short enough to feel immediate in practice. */
#define OUTPUT_SIZE_SETTLE_SECONDS 1.0f

/* Minimum real seconds between analysis passes while fast_recheck is
 * active. Without a cap, a source whose detected boundary doesn't settle
 * within 2 consecutive readings (not unusual -- that's the whole reason
 * debounce exists) would trigger a full GPU analysis pass (texrender +
 * a GPU->CPU readback + a 640x360 pixel scan) on every rendered frame,
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
    float    scan_interval;     /* seconds between analyses              */
    float    max_darkness;    /* 0-1: gate fires when zone luma < this     */
    float    edge_sample_y;   /* 0-1: how far in from top/bottom to scan   */
    float    edge_sample_x;   /* 0-1: how far in from left/right to scan   */
    int      default_crop;      /* DEFAULT_CROP_NONE/4_3/16_9               */
    volatile bool force_crop;          /* true: bypass gates and commit immediately (UI->video thread) */
    volatile bool needs_analysis;      /* true: run analysis this render frame  (UI->video thread)   */
    volatile bool fast_recheck;        /* true: use fast interval               (UI->video thread)   */
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
     * min_update_size on every edge, the candidate is discarded and the
     * existing committed crop is kept exactly as-is. Debounce protects
     * against trusting a single noisy reading; it does NOT protect
     * against the noise floor itself drifting -- two consecutive
     * readings can perfectly agree with each other at a position
     * that's still only noise relative to what's already committed,
     * and debounce alone has no way to tell the difference. This adds
     * that second comparison specifically. */
    bool     skip_minor_updates;
    float    min_update_size;  /* 0-1 fraction; differences below this on
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
    float crop_x0, crop_y0;   /* top-left  */
    float crop_x1, crop_y1;   /* bot-right */

    /* -- Pending RAW candidate (debounce buffer) -- */
    float pending_x0, pending_y0;
    float pending_x1, pending_y1;
    int   debounce_count;

    /* -- Is there a valid committed crop for this resolution? --
     * False on filter create and after a resolution change. Set to true
     * when the first debounce commit succeeds, or when Default Crop or
     * Force Crop Now commits directly. Guards skip_minor_updates so it
     * doesn't compare against uninitialised values before anything has
     * ever been committed. */
    bool crop_valid;

    /* -- Fast-recheck mode --
     * Set whenever the user explicitly changes a setting, presses
     * Force Crop Now, or the first valid reading arrives after
     * one or more dark/black-gated skips. While true, filter_tick
     * re-triggers analysis every FAST_RECHECK_INTERVAL_SECS instead of
     * waiting for the normal Recalc Interval timer, so debounce's
     * usual multi-sample confirmation resolves within a fraction of a
     * second rather than multiple full Recalc Interval cycles --
     * without skipping debounce itself, which still genuinely requires
     * `debounce_needed` consecutive matching reads, just gathered
     * faster. */
    float fast_recheck_elapsed; /* seconds since the last fast-recheck pass */

    /* -- Tracked source dimensions (detect resolution changes) -- */
    uint32_t last_src_w;
    uint32_t last_src_h;

    /* -- Debounced stable output size (runtime only, never saved) --
     * Used only when Output Width/Height = 0 (auto). A new live
     * resolution must hold steady for OUTPUT_SIZE_SETTLE_SECONDS before
     * being adopted -- this filters out brief negotiation blips during
     * capture device startup/mode changes without permanently locking
     * onto a possibly-wrong value the way a one-time snapshot would. */
    uint32_t adopted_out_w, adopted_out_h; /* currently adopted stable size */
    uint32_t candidate_w, candidate_h;   /* most recent live reading      */
    float    candidate_secs;               /* seconds candidate has held    */

    /* -- Timing / state flags -- */
    float elapsed;
    bool  needs_initial_scan;

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
 *   - The average luma of non-pure-black pixels is below max_darkness (dark / fade-to-black).
 *   - The detected content region is implausibly small (< 10 % per axis).
 * ============================================================================= */
/* Returns true if the pixel zone [x0,x1) x [y0,y1) passes the darkness gate.
 *
 * Pure-black pixels (max(r,g,b) <= 1) are excluded from the calculation --
 * RetroTink hardware borders are pure black and would otherwise dominate the
 * zone luma, causing the gate to fire on valid content frames that simply
 * have wide borders.
 *
 * The gate fires when the average luma of all remaining (non-pure-black)
 * pixels falls below max_darkness. A zone that is entirely pure-black (e.g.
 * a genuine black loading screen with no content at all) is always gated. */
static bool zone_passes_gates(const uint8_t *data, uint32_t linesize,
                               uint32_t x0, uint32_t y0,
                               uint32_t x1, uint32_t y1,
                               float max_darkness)
{
    double   luma_sum = 0.0;
    uint32_t total    = 0;

    for (uint32_t y = y0; y < y1; y++) {
        const uint8_t *row = data + y * linesize;
        for (uint32_t x = x0; x < x1; x++) {
            uint8_t b = row[x*4+0], g = row[x*4+1], r = row[x*4+2];
            uint8_t m = b > g ? b : g; if (r > m) m = r;
            if (m <= 1) continue;  /* skip pure black / 0x010101 */
            luma_sum += 0.114*b + 0.587*g + 0.299*r;
            total++;
        }
    }

    if (total == 0) return false;
    double luma = luma_sum / ((double)total * 255.0);
    return luma >= (double)max_darkness;
}

static bool analyse_frame(struct dynamic_autocrop_filter *f,
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

    /* -- Per-edge perimeter scan with brightness gating --
     *
     * Each edge has its own sampling zone:
     *   Top / Bottom  : edge_sample_y * H rows from the respective edge
     *   Left / Right  : edge_sample_x * W cols from the respective edge
     *
     * All four zones are checked against max_darkness. If ANY zone fails,
     * the entire analysis is held -- no partial updates. All zones must
     * pass for detection to proceed. */

#define IS_CONTENT(row, x) \
    ((row)[(x) * 4 + 0] > (uint8_t)T || \
     (row)[(x) * 4 + 1] > (uint8_t)T || \
     (row)[(x) * 4 + 2] > (uint8_t)T)

#define MIN_CONTENT_FRACTION 0.02f

    uint32_t min_count_w = (uint32_t)((float)W * MIN_CONTENT_FRACTION);
    if (min_count_w < 2) min_count_w = 2;
    uint32_t min_count_h = (uint32_t)((float)H * MIN_CONTENT_FRACTION);
    if (min_count_h < 2) min_count_h = 2;

    uint32_t zone_y = (uint32_t)(f->edge_sample_y * (float)H);
    uint32_t zone_x = (uint32_t)(f->edge_sample_x * (float)W);
    if (zone_y < 1) zone_y = 1;
    if (zone_y > H) zone_y = H;
    if (zone_x < 1) zone_x = 1;
    if (zone_x > W) zone_x = W;

    /* Gate each zone independently */
    bool top_ok = zone_passes_gates(data, linesize,
                                    0, 0, W, zone_y,
                                    f->max_darkness);
    bool bot_ok = zone_passes_gates(data, linesize,
                                    0, H - zone_y, W, H,
                                    f->max_darkness);
    bool lft_ok = zone_passes_gates(data, linesize,
                                    0, 0, zone_x, H,
                                    f->max_darkness);
    bool rgt_ok = zone_passes_gates(data, linesize,
                                    W - zone_x, 0, W, H,
                                    f->max_darkness);

    /* All four zones must pass for normal analysis.
     * If force_crop is set, bypass gate checks entirely regardless of
     * zone state -- the flag is left for do_analysis to consume and
     * commit immediately without debounce. */
    if (f->force_crop) {
        /* Gate bypass -- scan anyway, do_analysis will commit */
    } else if (!top_ok || !bot_ok || !lft_ok || !rgt_ok) {
        gs_stagesurface_unmap(f->stagesurf);
        return false;
    }

    /* Top edge */
    uint32_t top = zone_y;
    for (uint32_t y = 0; y < zone_y; y++) {
        const uint8_t *row = data + y * linesize;
        uint32_t count = 0;
        for (uint32_t x = 0; x < W; x++)
            if (IS_CONTENT(row, x)) count++;
        if (count >= min_count_w) { top = y; break; }
    }

    /* Bottom edge */
    uint32_t bot = H - 1 - zone_y;
    for (uint32_t i = 0; i < zone_y; i++) {
        uint32_t y = H - 1 - i;
        const uint8_t *row = data + y * linesize;
        uint32_t count = 0;
        for (uint32_t x = 0; x < W; x++)
            if (IS_CONTENT(row, x)) count++;
        if (count >= min_count_w) { bot = y; break; }
    }

    /* Left edge */
    uint32_t lft = zone_x;
    for (uint32_t x = 0; x < zone_x; x++) {
        uint32_t count = 0;
        for (uint32_t y = 0; y < H; y++) {
            const uint8_t *row = data + y * linesize;
            if (IS_CONTENT(row, x)) count++;
        }
        if (count >= min_count_h) { lft = x; break; }
    }

    /* Right edge */
    uint32_t rgt = W - 1 - zone_x;
    for (uint32_t i = 0; i < zone_x; i++) {
        uint32_t x = W - 1 - i;
        uint32_t count = 0;
        for (uint32_t y = 0; y < H; y++) {
            const uint8_t *row = data + y * linesize;
            if (IS_CONTENT(row, x)) count++;
        }
        if (count >= min_count_h) { rgt = x; break; }
    }

#undef IS_CONTENT
#undef MIN_CONTENT_FRACTION

    gs_stagesurface_unmap(f->stagesurf);

    float x0 = (float)lft        / (float)W;
    float y0 = (float)top        / (float)H;
    float x1 = (float)(rgt + 1)  / (float)W;
    float y1 = (float)(bot + 1)  / (float)H;

    if ((x1 - x0) < 0.02f || (y1 - y0) < 0.02f) {
        return false;
    }

    *out_x0 = x0;  *out_y0 = y0;
    *out_x1 = x1;  *out_y1 = y1;
    return true;
}

/* =============================================================================
 * Crop post-processing -- Max Crop X/Y, Horizontal/Vertical Trim
 * =============================================================================
 * Pure function: takes a raw detected boundary plus the filter's CURRENT
 * settings and produces the final crop to actually render. Deliberately
 * has no dependency on whether a fresh detection happened recently, no
 * gating, no rate-limiting, and no side effects -- it's meant to be
 * cheap enough to call on every single rendered frame, so that Horizontal/Vertical Trim and Max Crop X/Y always
 * reflect the latest value instantly, the moment they're changed,
 * completely independent of the (rate-limited, gated) detection pipeline
 * that produces the raw boundary it works from. This is what Max Crop
 * X/Y already needed fixed once before (a settings change had no visible
 * effect until detection happened to succeed again, which Max Darkness %
 * gating could block indefinitely) -- Horizontal/Vertical Trim had exactly
 * the same latent bug, just less obviously, since changing it usually
 * does still trigger a fast-tracked fresh detection most of the time,
 * masking the underlying issue except when gating got in the way.
 * ============================================================================= */
static void apply_crop_postprocessing(struct dynamic_autocrop_filter *f,
                                       float raw_x0, float raw_y0,
                                       float raw_x1, float raw_y1,
                                       uint32_t src_w, uint32_t src_h,
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
    }
    if (raw_height < min_height) {
        float deficit = (min_height - raw_height) * 0.5f;
        y0 = raw_y0 - deficit;
        y1 = raw_y1 + deficit;
    }

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

/* Compute the default crop position based on the selected mode and source
 * dimensions. Returns a centered crop that removes the expected bars:
 *   4:3 mode on 16:9 source → pillarbox removal (crop left/right)
 *   16:9 mode on 4:3 source → letterbox removal (crop top/bottom)
 *   Any mode where source already matches → full frame
 * This becomes the reset position whenever detection starts fresh. */
static void get_default_crop(int mode, uint32_t src_w, uint32_t src_h,
                              float *cx0, float *cy0,
                              float *cx1, float *cy1)
{
    *cx0 = 0.f; *cy0 = 0.f; *cx1 = 1.f; *cy1 = 1.f;
    if (mode == DEFAULT_CROP_NONE || src_w == 0 || src_h == 0) return;

    if (mode == DEFAULT_CROP_4_3) {
        /* Remove pillarbox: center a 4:3 region at full height */
        float target_w = (float)src_h * (4.f / 3.f);
        if (target_w < (float)src_w) {
            float margin = (1.f - target_w / (float)src_w) * 0.5f;
            *cx0 = margin; *cx1 = 1.f - margin;
        }
        /* If source is already 4:3 or narrower, leave full frame */
    } else if (mode == DEFAULT_CROP_16_9) {
        /* Remove letterbox: center a 16:9 region at full width */
        float target_h = (float)src_w * (9.f / 16.f);
        if (target_h < (float)src_h) {
            float margin = (1.f - target_h / (float)src_h) * 0.5f;
            *cy0 = margin; *cy1 = 1.f - margin;
        }
        /* If source is already 16:9 or wider, leave full frame */
    }
}

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
    if (!gs_texrender_begin(f->texrender, ANA_W, ANA_H)) {
        /* GPU resource temporarily unavailable -- back to normal cadence
         * rather than spinning in fast_recheck mode on every tick. */
        f->fast_recheck = false;
        f->debounce_count = 0;
        return;
    }

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
    if (!analyse_frame(f, &nx0, &ny0, &nx1, &ny1)) {
        /* Gates closed or frame degenerate -- back to normal 1s cadence
         * and reset debounce. Fast recheck only runs while gates are
         * actively open and debounce is in progress. */
        f->fast_recheck  = false;
        f->debounce_count  = 0;
        return;
    }

    /* Gates are open -- arm fast recheck so debounce confirmations
     * happen at ~100ms intervals rather than waiting a full second
     * between each reading. Turns off again once debounce completes. */
    f->fast_recheck         = true;
    f->fast_recheck_elapsed = 0.f;

    /* Clamp the detected crop to the user's selected default crop.
     * The default crop is the floor -- detection can only ever tighten
     * it, never widen past it. This means a blue no-signal screen or
     * blank frame that scans as full-frame simply returns the default
     * crop rather than blowing past it to 0,0,1,1. */
    if (f->default_crop != DEFAULT_CROP_NONE) {
        float dc0, dy0, dc1, dy1;
        get_default_crop(f->default_crop, src_w, src_h, &dc0, &dy0, &dc1, &dy1);
        if (nx0 < dc0) nx0 = dc0;
        if (ny0 < dy0) ny0 = dy0;
        if (nx1 > dc1) nx1 = dc1;
        if (ny1 > dy1) ny1 = dy1;
    }

    /* -- Commit logic --
     * Two things bypass debounce and commit immediately:
     *
     *   1. Force Crop Now button -- user explicitly requested it.
     *   2. (nothing else) -- first detection goes through normal debounce
     *      like everything else. Default Crop already shows a sensible
     *      position while debounce accumulates, so there is no reason to
     *      rush the first commit. */    /* -- Force Crop Now bypass --
     * If the button was pressed, skip all debounce and minor-update
     * checks and commit immediately. Clear the flag after commit. */
    if (f->force_crop) {
        f->crop_x0 = nx0; f->crop_y0 = ny0;
        f->crop_x1 = nx1; f->crop_y1 = ny1;
        f->debounce_count   = 0;
        f->fast_recheck   = false;
        f->force_crop     = false;
        f->crop_valid = true;
        blog(LOG_INFO, "[dynamic-autocrop] Force crop committed  L=%.4f T=%.4f R=%.4f B=%.4f",
             f->crop_x0, f->crop_y0, f->crop_x1, f->crop_y1);
        return;
    }

    /* -- Debounce (genuine, every time past the bootstrap case) --
     * Only commit a new crop after it has been stable for
     * `debounce_needed` consecutive analysis passes. This prevents a
     * single transitional frame (cut, wipe, flash, or a noisy one-off
     * reading) from snapping the crop to a bad value. fast_recheck
     * just controls how quickly those consecutive passes are gathered.
     *
     * Early exit: if the candidate is already too close to the committed
     * crop to be worth committing, don't bother debouncing -- bail
     * immediately and go back to 1s cadence. */
    if (f->crop_valid && f->skip_minor_updates &&
        crops_match(nx0, ny0, nx1, ny1,
                    f->crop_x0, f->crop_y0, f->crop_x1, f->crop_y1, f->min_update_size)) {
        f->debounce_count  = 0;
        f->fast_recheck  = false;
        return;
    }

    if (crops_match(nx0, ny0, nx1, ny1,
                    f->pending_x0, f->pending_y0, f->pending_x1, f->pending_y1, CROP_TOLERANCE)) {
        f->debounce_count++;
        /* Still track the LATEST reading even on a match, not just the
         * first-in-streak value. Without this, while a setting is being
         * actively adjusted (e.g. dragging the Horizontal/Vertical Trim slider --
         * each frame's reading differs slightly from the last but still
         * falls within tolerance of the streak's starting point), the
         * eventual commit applies a stale value from whenever the
         * streak began rather than wherever the slider actually ended
         * up, making the crop visibly lag behind the live setting. */
        f->pending_x0 = nx0; f->pending_y0 = ny0;
        f->pending_x1 = nx1; f->pending_y1 = ny1;
    } else {
        /* New candidate -- restart countdown */
        f->pending_x0 = nx0; f->pending_y0 = ny0;
        f->pending_x1 = nx1; f->pending_y1 = ny1;
        f->debounce_count = 1;
    }

    if (f->debounce_count >= f->debounce_needed) {
        /* Debounce confirmed the candidate is stable across consecutive
         * readings. Final check: is it actually different enough from the
         * committed crop to be worth applying? (The early-exit above
         * handles the common case; this catches candidates that started
         * larger but settled within threshold by the time debounce ran.) */
        if (f->skip_minor_updates &&
            crops_match(f->pending_x0, f->pending_y0, f->pending_x1, f->pending_y1,
                        f->crop_x0, f->crop_y0, f->crop_x1, f->crop_y1, f->min_update_size)) {
            /* Below threshold on every edge -- treat as noise, keep the
             * committed crop exactly as it is. */
            f->debounce_count      = 0;
            f->fast_recheck      = false;

            return;
        }

        f->crop_x0 = f->pending_x0; f->crop_y0 = f->pending_y0;
        f->crop_x1 = f->pending_x1; f->crop_y1 = f->pending_y1;
        f->debounce_count      = 0;
        f->fast_recheck      = false;
        f->crop_valid    = true;
        blog(LOG_INFO, "[dynamic-autocrop] Crop updated  L=%.4f T=%.4f R=%.4f B=%.4f",
             f->crop_x0, f->crop_y0, f->crop_x1, f->crop_y1);

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
    obs_data_set_default_int   (settings, S_BLACK_THRESH,  45);
    obs_data_set_default_double(settings, S_TRIM_X,        0.4);
    obs_data_set_default_double(settings, S_TRIM_Y,        0.4);
    obs_data_set_default_double(settings, S_RECALC_SECS,   1.0);
    obs_data_set_default_double(settings, S_MAX_DARKNESS,  6.0);
    obs_data_set_default_int   (settings, S_DEFAULT_CROP,  DEFAULT_CROP_NONE);
    obs_data_set_default_int   (settings, S_EDGE_SAMPLE_Y, 12);
    obs_data_set_default_int   (settings, S_EDGE_SAMPLE_X, 20);
    obs_data_set_default_int   (settings, S_DEBOUNCE,      4);
    obs_data_set_default_bool  (settings, S_FREEZE_CROP,   false);
    obs_data_set_default_int   (settings, S_MAX_CROP_X,    60);
    obs_data_set_default_int   (settings, S_MAX_CROP_Y,    30);
    obs_data_set_default_bool  (settings, S_LIMIT_SMALL_CHANGES, true);
    obs_data_set_default_double(settings, S_MIN_CHANGE_PCT,      3.0);
}

/* -- Update (settings changed) -- */
static void filter_update(void *data, obs_data_t *settings)
{
    struct dynamic_autocrop_filter *f = data;

    int  prev_default_crop = f->default_crop;
    f->out_w_setting  = (uint32_t)obs_data_get_int(settings, S_OUT_W);
    f->out_h_setting  = (uint32_t)obs_data_get_int(settings, S_OUT_H);
    f->scale_mode     = (int)     obs_data_get_int(settings, S_SCALE_FILTER);
    f->black_thresh   = (int)     obs_data_get_int(settings, S_BLACK_THRESH);
    f->trim_x         = (float)obs_data_get_double(settings, S_TRIM_X) / 100.f;
    f->trim_y         = (float)obs_data_get_double(settings, S_TRIM_Y) / 100.f;
    f->scan_interval    = (float)obs_data_get_double(settings, S_RECALC_SECS);
    f->max_darkness     = (float)obs_data_get_double(settings, S_MAX_DARKNESS)  / 100.f;
    f->edge_sample_y    = (float)obs_data_get_int(settings, S_EDGE_SAMPLE_Y) / 100.f;
    f->edge_sample_x    = (float)obs_data_get_int(settings, S_EDGE_SAMPLE_X) / 100.f;
    f->default_crop      = (int)obs_data_get_int(settings, S_DEFAULT_CROP);
    f->debounce_needed= (int)     obs_data_get_int(settings, S_DEBOUNCE);
    f->freeze_crop    = obs_data_get_bool(settings, S_FREEZE_CROP);
    f->max_crop_x     = (float)obs_data_get_int(settings, S_MAX_CROP_X) / 100.f;
    f->max_crop_y     = (float)obs_data_get_int(settings, S_MAX_CROP_Y) / 100.f;
    f->skip_minor_updates = obs_data_get_bool(settings, S_LIMIT_SMALL_CHANGES);
    f->min_update_size      = (float)obs_data_get_double(settings, S_MIN_CHANGE_PCT) / 100.f;

    /* Clamp recalc to a sane minimum to avoid GPU hammering */
    if (f->scan_interval < 0.5f)
        f->scan_interval = 0.5f;

    /* Threading note: the three volatile flags (force_crop, needs_analysis,
     * fast_recheck) are the only fields written here that are also read on
     * the video thread mid-frame. All other settings fields above are only
     * read by filter_tick / filter_render at the START of a new frame, after
     * OBS has serialized the update callback -- OBS guarantees filter_update
     * completes before the next video frame begins, so no mutex is needed
     * for those fields. The volatile flags are the exception because they
     * can be set asynchronously from button callbacks at any time. */

    /* If the user changed the Default Crop dropdown, immediately apply it
     * and commit -- bypass all gates, debounce, and dark-frame checks.
     * We need src dimensions for this; get them from the filter target. */
    if (f->default_crop != prev_default_crop) {
        obs_source_t *target = obs_filter_get_target(f->context);
        uint32_t sw = target ? obs_source_get_base_width(target)  : 0;
        uint32_t sh = target ? obs_source_get_base_height(target) : 0;
        if (sw > 0 && sh > 0) {
            float dc0, dy0, dc1, dy1;
            get_default_crop(f->default_crop, sw, sh, &dc0, &dy0, &dc1, &dy1);
            f->crop_x0 = dc0; f->crop_y0 = dy0;
            f->crop_x1 = dc1; f->crop_y1 = dy1;
            f->pending_x0 = dc0; f->pending_y0 = dy0;
            f->pending_x1 = dc1; f->pending_y1 = dy1;
            f->crop_valid    = true;
            f->debounce_count      = 0;
            blog(LOG_INFO, "[dynamic-autocrop] Default crop changed -- committed L=%.4f T=%.4f R=%.4f B=%.4f",
                 f->crop_x0, f->crop_y0, f->crop_x1, f->crop_y1);
        }
    }

    /* Trigger a fresh, fast-tracked analysis so setting changes take
     * effect quickly -- but not while frozen, since freezing
     * specifically means "don't change the crop"; the Recalculate Crop
     * Now button is the explicit escape hatch for that. fast_recheck
     * makes confirmation happen within a fraction of a second instead
     * of a full Recalc Interval, without skipping debounce's agreement
     * requirement, so a single noisy reading still can't commit on its
     * own. */
    if (!f->freeze_crop) {
        os_atomic_store_bool(&f->fast_recheck,   true);
        os_atomic_store_bool(&f->needs_analysis, true);
    }
}

/* -- Force Crop Now button -- bypasses darkness gates on the next pass -- */
static bool filter_recalc_button_clicked(obs_properties_t *props, obs_property_t *property, void *data)
{
    UNUSED_PARAMETER(props);
    UNUSED_PARAMETER(property);
    struct dynamic_autocrop_filter *f = data;
    os_atomic_store_bool(&f->fast_recheck,   true);
    os_atomic_store_bool(&f->needs_analysis, true);
    os_atomic_store_bool(&f->force_crop,     true);
    return false;
}

/* -- Toggle Min Change % slider visibility live as Ignore Small Changes is checked/unchecked -- */
static bool filter_skip_minor_updates_modified(obs_properties_t *props, obs_property_t *property, obs_data_t *settings)
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
    f->crop_x0 = 0.f; f->crop_y0 = 0.f;
    f->crop_x1 = 1.f; f->crop_y1 = 1.f;
    f->pending_x0 = 0.f; f->pending_y0 = 0.f;
    f->pending_x1 = 1.f; f->pending_y1 = 1.f;

    f->crop_valid = false;

    f->needs_initial_scan       = true;
    f->needs_analysis   = false;
    f->elapsed          = 0.f;
    f->debounce_count      = 0;
    f->last_src_w        = 0;
    f->last_src_h        = 0;

    /* Debounced stable-output-size tracking starts empty; filter_tick
     * populates it from the live source over the next few ticks. See
     * the struct field comments and filter_tick for the settle-time
     * logic this uses. */
    f->adopted_out_w = 0;
    f->adopted_out_h = 0;
    f->candidate_w  = 0;
    f->candidate_h  = 0;
    f->candidate_secs = 0.f;

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
     * tick regardless of needs_initial_scan/freeze state, since it's about
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
                f->candidate_secs = 0.f;
            } else {
                f->candidate_secs += t;
            }

            bool no_stable_yet = (f->adopted_out_w == 0 || f->adopted_out_h == 0);
            if (no_stable_yet || f->candidate_secs >= OUTPUT_SIZE_SETTLE_SECONDS) {
                f->adopted_out_w = f->candidate_w;
                f->adopted_out_h = f->candidate_h;
            }
        }
    }

    /* First frame after creation: analyse immediately. This happens even
     * if Freeze Crop is on, since freezing is about holding a crop once
     * we HAVE one -- a brand new filter still needs an initial detection
     * rather than sitting uncropped forever. */
    if (f->needs_initial_scan) {
        f->needs_initial_scan      = false;
        f->needs_analysis = true;
        return;
    }

    /* While frozen, the periodic timer is suppressed entirely -- the
     * committed crop just holds. Force Crop Now is the only thing
     * that can trigger another analysis pass while frozen. */
    if (f->freeze_crop)
        return;

    /* Real elapsed time accumulates while not frozen -- reset wherever
     * a commit happens in do_analysis. Tracked here (real per-tick
     * delta time) rather than approximated per analysis call, since
     * fast_recheck can make do_analysis run many times per second. */

    if (f->fast_recheck) {
        /* A settings change, the Force Crop Now button, or the first
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
    if (f->elapsed >= f->scan_interval) {
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
    return f->adopted_out_w;
}

static uint32_t filter_get_height(void *data)
{
    struct dynamic_autocrop_filter *f = data;
    if (f->out_h_setting != 0)
        return f->out_h_setting;
    return f->adopted_out_h;
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

    /* If the source resolution changed, reset to default crop position and
     * force a fresh analysis on the new resolution. */
    if (src_w != f->last_src_w || src_h != f->last_src_h) {
        get_default_crop(f->default_crop, src_w, src_h,
                         &f->crop_x0, &f->crop_y0, &f->crop_x1, &f->crop_y1);
        f->debounce_count   = 0;
        f->crop_valid = false;
        f->needs_analysis = true;
        f->last_src_w     = src_w;
        f->last_src_h     = src_h;
        blog(LOG_INFO, "[dynamic-autocrop] Resolution changed to %ux%u -- resetting", src_w, src_h);
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
    uint32_t out_w = (f->out_w_setting != 0) ? f->out_w_setting : (f->adopted_out_w ? f->adopted_out_w : src_w);
    uint32_t out_h = (f->out_h_setting != 0) ? f->out_h_setting : (f->adopted_out_h ? f->adopted_out_h : src_h);

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
     * f->crop_x0..f->crop_y1 hold the RAW debounce-confirmed detected boundary
     * only -- Horizontal/Vertical Trim and Max Crop X/Y are applied
     * fresh, every single render frame, directly against whatever the
     * CURRENT settings are. This is what guarantees Horizontal/Vertical
     * Trim and Max Crop X/Y always take effect the instant they're
     * changed, with zero dependency on whether a new raw detection has
     * happened to run -- which Max Darkness % gating, or
     * simply Recalc Interval's pacing, could otherwise delay
     * indefinitely. f->crop_x0..f->crop_y1 themselves stay untouched here, since
     * debounce in do_analysis needs that genuine raw detection history
     * to compare fresh readings against. */
    float rx0, ry0, rx1, ry1;
    apply_crop_postprocessing(f, f->crop_x0, f->crop_y0, f->crop_x1, f->crop_y1, src_w, src_h,
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
    struct dynamic_autocrop_filter *f = data; /* may be NULL if called before create */
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

    obs_property_t *dc = obs_properties_add_list(props, S_DEFAULT_CROP,
        "Default Crop",
        OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);
    obs_property_list_add_int(dc, "None (full frame)",      DEFAULT_CROP_NONE);
    obs_property_list_add_int(dc, "4:3 (remove pillarbox)", DEFAULT_CROP_4_3);
    obs_property_list_add_int(dc, "16:9 (remove letterbox)", DEFAULT_CROP_16_9);
    obs_property_set_long_description(dc,
        "Starting crop position used when detection resets.\n"
        "4:3: centers a 4:3 region at full height -- removes pillarboxes on a 16:9 source.\n"
        "16:9: centers a 16:9 region at full width -- removes letterboxes on a 4:3 source.\n"
        "If the source already matches the selected ratio, no pre-crop is applied.");

    p = obs_properties_add_int_slider(props, S_BLACK_THRESH, "Black Threshold", 0, 128, 1);
    obs_property_set_long_description(p,
        "Per-pixel brightness cutoff for border detection. A pixel where max(R,G,B) is at or "
        "below this value is treated as black border; above it counts as content.\n"
        "0 = only pure black is border. 45 (default) = anything darker than ~18% brightness. "
        "128 = anything darker than 50% brightness.\n"
        "Raise for noisy or composite captures where the border isn't perfectly black.");

    p = obs_properties_add_float_slider(props, S_TRIM_X, "Horizontal Trim %", 0.0, 10.0, 0.1);
    obs_property_set_long_description(p, "Extra inward trim from the left and right edges, beyond the detected border.\nUse for stray garbage pixels just outside the picture (e.g. Mega Drive/Genesis dots).\nDefaults to a small non-zero value to prevent a thin border sliver -- lowering it toward 0% trades that protection away.");

    p = obs_properties_add_float_slider(props, S_TRIM_Y, "Vertical Trim %", 0.0, 10.0, 0.1);
    obs_property_set_long_description(p, "Extra inward trim from the top and bottom edges, beyond the detected border.\nUse for stray garbage pixels just outside the picture.\nDefaults to a small non-zero value to prevent a thin border sliver -- lowering it toward 0% trades that protection away.");

    p = obs_properties_add_int_slider(props, S_MAX_CROP_X, "Max Crop X %", 0, 90, 1);
    obs_property_set_long_description(p, "Never crop away more than this much of the frame's WIDTH, even if detection finds more.\nProtects genuine in-game black bars (e.g. TATE-mode vertical shooters like Ikaruga, played pillarboxed) from being over-cropped.");

    p = obs_properties_add_int_slider(props, S_MAX_CROP_Y, "Max Crop Y %", 0, 90, 1);
    obs_property_set_long_description(p, "Never crop away more than this much of the frame's HEIGHT, even if detection finds more.\nProtects genuine in-game black bars (e.g. title screens like Metroid Prime) from being over-cropped.");

    p = obs_properties_add_float_slider(props, S_RECALC_SECS, "Scan Interval (s)", 0.5, 10.0, 0.5);
    obs_property_set_long_description(p, "How often (seconds) to re-scan for the border.");

    p = obs_properties_add_float_slider(props, S_MAX_DARKNESS,
        "Max Darkness %", 0.0, 50.0, 0.5);
    obs_property_set_long_description(p,
        "Each edge's sampling zone must have an average luma (of all non-pure-black pixels) "
        "at or above this value to update that edge.\n"
        "Pure-black pixels (0x000000 and 0x010101 hardware border) are excluded from "
        "the calculation entirely -- only actual content and noise are measured.\n"
        "Lower = more permissive (allows darker zones through). "
        "Raise to hold the crop during dark scenes like loading screens or fade-outs.");

    p = obs_properties_add_int_slider(props, S_EDGE_SAMPLE_Y,
        "Edge Sampling Region Y %", 5, 50, 1);
    obs_property_set_long_description(p,
        "How far in from the top and bottom edges the brightness gate samples.\n"
        "Raise if your source has deep borders that keep the top/bottom zones gated.");

    p = obs_properties_add_int_slider(props, S_EDGE_SAMPLE_X,
        "Edge Sampling Region X %", 5, 50, 1);
    obs_property_set_long_description(p,
        "How far in from the left and right edges the brightness gate samples.\n"
        "Raise if your source has wide borders that keep the left/right zones gated.");

    p = obs_properties_add_int_slider(props, S_DEBOUNCE, "Debounce", 1, 10, 1);
    obs_property_set_long_description(p, "Stable samples required before applying a CHANGED crop.");

    p = obs_properties_add_bool(props, S_LIMIT_SMALL_CHANGES, "Skip Minor Updates");
    obs_property_set_long_description(p, "Skip applying a new crop if it's only marginally different from the current one -- helps avoid tiny adjustments caused by analog noise.");
    obs_property_set_modified_callback(p, filter_skip_minor_updates_modified);

    p = obs_properties_add_float_slider(props, S_MIN_CHANGE_PCT, "Minimum Update Size %", 0.1, 5.0, 0.1);
    obs_property_set_long_description(p, "How different (per edge) a new crop must be from the current one before it's treated as a real change rather than noise.");
    obs_property_set_visible(p, f ? f->skip_minor_updates : false);

    p = obs_properties_add_bool(props, S_FREEZE_CROP, "Freeze Crop");
    obs_property_set_long_description(p, "Stop re-scanning and hold the current crop exactly as-is.\nUseful as a workaround for games that otherwise confuse detection.");

    obs_properties_add_button(props, S_RECALC_BUTTON,
        "Force Crop Now", filter_recalc_button_clicked);

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
