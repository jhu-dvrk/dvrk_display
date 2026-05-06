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

**How:** The user enables the calibration grid (`g` key in the calibration tool, or `--grid` flag in the stereo program). The grid is a 3×4 array of coloured squares, centred per-eye with an offset of `±display_horizontal_offset_px/2`. Looking through the stereo display, the user moves their hand to the working depth, focuses on it, and adjusts `[`/`]` until the grid squares are fused (not doubled).

**What the calibration tool does:**

The grid center for each eye is placed at:

```
cx = (crop_w / 2) + sign × display_horizontal_offset_px / 2
```

where sign = −1 for left eye, +1 for right eye, and `crop_w` is the **crop-frame width** (e.g. 496 px). The grid is thus drawn directly on the cropped (non-upscaled) preview images.

**What the stereo pipeline does:**

- `glvideomixer` places the upscaled left eye video at `left_xpos` and the right eye at `right_xpos`:

  ```
  horizontal_ui_scale = (original_w − 2 × |offset|) / original_w
  shifted_eye_w       = round(original_w × horizontal_ui_scale)
  left_xpos           = original_w/2 − offset/2 − shifted_eye_w/2
  right_xpos          = original_w + original_w/2 + offset/2 − shifted_eye_w/2
  ```

  The video is simultaneously shifted inward and slightly compressed horizontally so that no content is clipped at the screen edge.

- `overlay.cpp` (`on_overlay_draw`) draws HUD icons with the same formula, but in **original-pixel space**:

  ```
  left_baseline_cx  = original_w/2 − display_horizontal_offset_px/2
  right_baseline_cx = original_w + original_w/2 + display_horizontal_offset_px/2
  ```

---

## Known Coordinate-Space Discrepancy

**The `display_horizontal_offset_px` value is calibrated in crop-pixel space but applied in original-pixel space.**

| Quantity | Calibration tool | Stereo pipeline |
|---|---|---|
| Eye width used for grid/video centre | `crop_w` (e.g. 496 px) | `original_w` (e.g. 640 px) |
| Shift applied per eye | `offset / 2` px of a 496 px frame | `offset / 2` px of a 640 px frame |
| Disparity as % of eye width | `offset / crop_w` (≈ 14.9 % for offset=74) | `offset / original_w` (≈ 11.6 %) |

If the calibration window and the stereo display window are shown at the same physical size, the stereo app will produce ~22 % less angular disparity than what was calibrated. The working depth in the stereo viewer will therefore appear farther than intended.

**Correct fix:** When saving `display_horizontal_offset_px` from the calibration tool, scale it by `original_w / crop_w` so that the value stored is in original-pixel space:

```python
saved_offset = int(round(display_horizontal_offset * original_w / crop_w))
```

Alternatively, store the offset as a fraction of eye width and convert to pixels in each context.

---

## `glimage` vs `glimages`

- **`glimage`** — one GTK window shows the full `2 × original_w` side-by-side frame. A single `stereo_overlay` element draws HUD on the combined frame with the full stereo layout (left and right eye icons at different horizontal positions, producing stereo depth).
- **`glimages`** — two GTK windows, one per eye. Each has its own `left_overlay` / `right_overlay`. Currently these overlays use `OverlayView::LeftEye` / `OverlayView::RightEye` and draw icons at the single-eye frame centre — **no display offset is applied**, so the HUD has zero disparity. This will need a fix if the per-eye overlay should appear at working depth.

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
