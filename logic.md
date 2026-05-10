# dvrk_display Stereo Calibration Logic

## Overview

Calibration is a two-step process, each step producing one parameter saved to the JSON config.

---

## Step 1 — Camera alignment (`camera.alignment`)

**Goal:** Make a physical calibration cross (printed at a fixed, known depth from the endoscope tip) appear at zero disparity — i.e., the left and right camera images of the cross fuse perfectly.

**How:** The calibration tool overlays the left and right frames on top of each other with different colour tints (one is green, the other is magenta). The user adjusts `horizontal_shift_px` (and optionally `vertical_shift_px`) with the arrow keys until the tinted overlay becomes neutral (the cross fuses and the colour cancels).

**What the code does (calibration tool):**

- Each camera produces a raw frame of size `original_w × original_h` (e.g. 640×480).
- `compute_eye_crop()` maps `horizontal_shift_px` and `vertical_shift_px` to asymmetric crop windows for the left eye (sign = −1) and right eye (sign = +1). A positive horizontal shift moves the left window to the right and the right window to the left (convergence); negative diverges them.
- The crop reduces the frame to `crop_w × crop_h` (e.g. 496×372).
- Only the overlapping content (within both crop windows) remains, which is what gives alignment. Everything outside the intersection is discarded.

**What the stereo pipeline does:**

- `videocrop` applies the computed left/right crop values per eye.
- Because `preserve_size = true`, `videoscale` then scales the cropped frame back up to `original_w × original_h`. This scale-up is what ensures that all downstream systems (different hardware) receive a consistent frame size.
- Because the aspect ratio of the cropped frame may differ slightly from the original (when both h and w are cropped), an additional `aspect_crop` is applied inside `build_pipeline_string` before the `videoscale`, so the output truly fills `original_w × original_h` without pillar/letterboxing.

**Saved as:** `camera.alignment.horizontal_shift_px`, `camera.alignment.vertical_shift_px`

---

## Step 2 — Display alignment (`display_horizontal_offset_px`)

**Goal:** Find the horizontal pixel shift that makes the stereo display present content at the depth where the surgeon's hands typically operate.

**How:** The user enables the calibration grid (`g` key). The grid is centred per-eye at `cx = (crop_w / 2) + sign × display_horizontal_offset_px / 2` (sign = −1 for left, +1 for right). The user adjusts `[`/`]` until the grid squares are fused when looking at the working depth.

**What the stereo pipeline does:**

The display offset is applied as an additional **asymmetric crop** of each eye's video, folded into the same `videocrop` element that already carries the camera alignment crop:

- **Left eye**: crop window shifts left by `display_horizontal_offset_px / 2` pixels (show more outer/left content, less inner/right content).
- **Right eye**: crop window shifts right by the remaining `display_horizontal_offset_px − display_horizontal_offset_px/2` pixels.
- Total width removed from each eye stays the same, so `videoscale` still upscales to exactly `original_w × original_h`.
- Both eye windows are filled edge-to-edge — **no black bars, no bleed** between windows.

The `glvideomixer` is a plain side-by-side compositor (left at `xpos=0`, right at `xpos=eye_w`, both full `eye_w` wide).

**Overlay (Cairo HUD):** The display offset is applied directly to icon positions in pixel space:
- For the combined stereo view (`glimage`): `left_cx = eye_w/2 − offset/2`, `right_cx = 3·eye_w/2 + offset/2`.
- For per-eye windows (`glimages`): each overlay draws at `cx = eye_w/2 + sign × offset/2`.

**Saved as:** `display_horizontal_offset_px`

---

## Coordinate-Space Note

The `display_horizontal_offset_px` is calibrated in **crop-pixel space** (the preview frames in the calibration tool are at `crop_w × crop_h`). The stereo pipeline applies the offset directly in **original-pixel space** (post-upscale crop fields). For typical zoom levels the difference is small, but for large crop ratios the offset could differ. If precise depth calibration is needed, store the offset as a fraction of eye width and convert to pixels in each context.

---

## `glimage` vs `glimages`

- **`glimage`** — one GTK window shows the full `2 × original_w` side-by-side frame. A single `stereo_overlay` element draws HUD on the combined frame with the full stereo layout.
- **`glimages`** — two GTK windows, one per eye. The display offset is baked into each eye's `videocrop` window so both monitors show edge-to-edge content at the correct convergence depth. Each eye's overlay (`left_overlay` / `right_overlay`) draws icons shifted by `±offset/2` from the single-eye frame centre, matching the video depth.

---

## Config Reference

```json
{
  "camera": {
    "size":      { "width": 640, "height": 480 },
    "left":      { "stream": "...", "color": { ... } },
    "right":     { "stream": "...", "color": { ... } },
    "crop":      { "width": 496, "height": 372 },
    "alignment": { "horizontal_shift_px": -143, "vertical_shift_px": 0 }
  },
  "preserve_size": true,
  "display_horizontal_offset_px": -74,
  "sinks": ["glimage"]
}
```

| Field | Set by | Step |
|---|---|---|
| `camera.alignment.*` | Calibration tool (arrow keys) | Step 1 |
| `camera.crop.*` | Calibration tool (+/- keys) | Step 1 |
| `camera.*.color` | Calibration tool (c key) | Optional colour match |
| `display_horizontal_offset_px` | Calibration tool ([ / ] keys) | Step 2 |
