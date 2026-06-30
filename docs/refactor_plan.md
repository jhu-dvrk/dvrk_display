# dVRK Data and Console Refactor Plan

## Goals

Split the current `dvrk_display` package into two ROS 2 packages with clearer
ownership:

- `dvrk_data`: video acquisition, GStreamer transport, processing,
  timestamp metadata, recording, playback, and reusable data utilities.
- `dvrk_console`: surgeon/operator-facing applications such as stereo display,
  display controls, overlays, and interactive calibration tools.

The dependency direction should be one-way:

```text
dvrk_console -> dvrk_data
```

`dvrk_data` must not depend on `dvrk_console`.

## Package Responsibilities

### dvrk_data

Owns reusable non-UI data and video processing code:

- GStreamer helper utilities and transport contracts.
- Custom timestamp metadata and propagation helpers.
- Reconnectable source/sink building blocks.
- Stereo rectification and normalization.
- Stereo collation into side-by-side frames.
- Video recording and playback, including migrated `data_collection` code.
- Synthetic/test video sources.

Expected applications:

- `stereo_rectify`
- `stereo_collate`
- `video_record`
- `video_playback`
- `synthetic_source`
- AR or PIP test sources when they are data-stream generators.

### dvrk_console

Owns interactive console applications and presentation code:

- `stereo_display`
- `control_panel`
- `stereo_calibrate`
- Cairo/HUD overlay rendering.
- Display-window and monitor-placement controls.
- Console-specific launch files and user-facing configs.

## Video Pipeline Direction

The target runtime shape is:

```text
raw left/right streams
  -> optional stereo_rectify
  -> stereo_collate
  -> stereo_display
```

Rectification should stay upstream of collation where possible, because crop,
horizontal/vertical alignment, color correction, and future distortion
correction are per-eye camera operations.

`stereo_display` should consume an already-collated side-by-side stereo stream
plus optional AR/PIP streams. It should own presentation and layout, but not
camera geometry.

## Timestamp Contract

Keep frame timestamps in reusable `dvrk_data` code. Timestamp metadata should
be treated as an explicit transport contract and tested across process
boundaries such as `unixfdsink`/`unixfdsrc`.

For latency and correlation, carry both:

- absolute epoch timestamp for logging/cross-process correlation;
- monotonic timestamp for latency and duration measurements;
- frame id and source id where practical.

## Migration Order

1. Create package skeletons for `dvrk_data` and `dvrk_console`.
2. Move common config/GStreamer/timestamp helpers into `dvrk_data`.
3. Move overlay, display output controls, `stereo_display`, and `control_panel`
   into `dvrk_console`.
4. Use the new executable names directly: `stereo_display` and
   `stereo_calibrate`.
5. Extract `stereo_rectify` and `stereo_collate` from the current monolithic
   `stereo` pipeline.
6. Migrate existing `data_collection` code into `dvrk_data` and remove the
   external package dependency. Done in the initial package split: `dvrk_data`
   now builds the recording, extraction, video tagging, latency, configurator,
   shared timestamp metadata, and GStreamer utility features locally.
7. Add reconnect and metadata-preservation tests for interprocess video links.

## Compatibility Notes

The current package name, executable names, config type strings, socket paths,
and launch files are user-facing. Rename them in stages and keep shims while
downstream launch files are updated.
