# ofxDVS — Claude Code Project Guide

## What is this?

openFrameworks addon for neuromorphic (event-based) vision cameras. Interfaces with DVS128, DAVIS, and DVXplorer sensors via dv-processing/libcaer. Includes real-time neural network inference (YOLO object detection, TSDT gesture recognition), rectangular cluster tracking, and comprehensive visualization.

## Project Layout

```
src/
  ofxDVS.hpp / .cpp          — Main addon class (setup/update/draw lifecycle, ~2100 lines)
  dvs_yolo_pipeline.hpp/.cpp  — YOLO object detection (5-ch VTEI tensor)
  dvs_tsdt_pipeline.hpp/.cpp  — TSDT gesture recognition (T×2×H×W binary tensor)
  onnx_run.hpp/.cpp           — ONNX Runtime wrapper (float16/32, multi-thread)
  dvs_inference_worker.hpp    — Template async inference worker (non-blocking)
  dvs_nn_utils.hpp            — Header-only: sigmoid, IoU, NMS, letterbox transforms
  dvs_gui.hpp/.cpp            — GUI panel creation (NN panel, tracker panel)
  RectangularClusterTracker.hpp/.cpp — Event-driven multi-object tracker (~2200 lines)
  ofxDvsPolarity.hpp          — Polarity event struct
libs/
  libcaer/                    — iniVation camera C/C++ driver (bundled)
  onnxruntime/                — ONNX Runtime 1.20.0 shared libs (bundled)
example_dvs/                  — Minimal example app (ofApp delegates to ofxDVS)
  bin/data/                   — ONNX models and sample recordings
```

## Build

```bash
cd example_dvs && make -j$(nproc)
# or: make run
```

Uses openFrameworks 0.12.0 build system (`compile.project.mk`). Compiler: g++-13 with `-std=gnu++20`.

### Dependencies
- **openFrameworks 0.12.0** — windowing, GL, input
- **ofxDatGui** — advanced GUI panels (external addon in OF addons dir)
- **ofxPoco, ofxGui, ofxNetwork** — bundled OF addons
- **dv-processing** — modern camera I/O (system library)
- **libcaer** — low-level camera driver (bundled in libs/)
- **ONNX Runtime 1.20.0** — neural network inference (bundled in libs/)
- **OpenCV 4** — image processing (system)
- **Eigen3** — linear algebra (system)

## Architecture

### Threading Model
1. **USB Thread** (`usbThread : public ofThread`) — reads camera/file events, pushes `caerEventPacketContainer` to shared queue
2. **Main Thread** (OF lifecycle) — dequeues events in `update()`, processes, renders in `draw()`
3. **Async Inference Workers** (`InferenceWorker<T>`) — two instances (YOLO, TSDT), skip-frame semantics (no blocking)

### Event Processing Pipeline (in update())
1. Dequeue packets from USB thread → `organizeData()` → fills `packetsPolarity`
2. `updateBAFilter()` — background activity noise filter
3. `applyHotPixelFilter_()` — hot-pixel suppression (3 stages: calibration mask, refractory, rate-based)
4. `updateImageGenerator()` — accumulates spikes into image
5. Feed cluster tracker, YOLO pipeline, TSDT pipeline
6. All downstream consumers check `e.valid` flag

### Hot-Pixel Filter (private in ofxDVS)
- **Startup calibration**: counts per pixel for first N seconds, marks outliers (mean + sigma*stddev)
- **Refractory**: drops events arriving within `hot_refrac_us` of last event at same pixel
- **Rate-based**: drops pixels exceeding `hot_rate_threshold` events per `hot_rate_window_us` window
- GUI: sliders for refractory, rate window, rate threshold + "Recalibrate Hot Pixels" button

### GUI
- `f1` — main panel (ofxDatGui): toggles, sliders, buttons for DVS/APS/IMU, recording, filters, playback speed
- `nn_panel` — NN controls: YOLO config, TSDT config, TP Detector
- `optflow_panel` — optical flow + event reconstruction controls
- `tracker_panel` — 50+ cluster tracker parameters
- Event handlers: `onButtonEvent()`, `onToggleEvent()`, `onSliderEvent()`, `onMatrixEvent()`

## Key Patterns

- **Camera selection**: `#define DVXPLORER 1` in ofxDVS.hpp sets active camera family
- **Polarity events**: `struct polarity { ofVec2f pos; bool pol; int64_t timestamp; bool valid; }`
- **Valid flag**: All filters set `e.valid = false`; all consumers skip invalid events
- **Async NN**: Submit lambda job → worker executes → main reads `lastResult()` next frame
- **VTEI tensor**: 5-channel (pos count, neg count, time-surface, edge, intensity) for YOLO
- **File format**: AEDAT4 (dv-processing) for recording; AEDAT 3.1 and AEDAT4 for playback
- **Recording**: `dv::io::MonoCameraWriter` writes `.aedat4` files with event-only config
- **Playback timing**: file-time-to-wall-time mapping via `fileTimeOrigin_`/`wallTimeOrigin_`/`playbackSpeed_`; backpressure on reader thread prevents packet loss at slow speeds
- **Speed control**: log-scale slider (-1..+2 → 0.1x..100x); continuity on speed change and pause/resume

## Git

- **Remotes**: `origin` → gitlab.tue.nl (primary), `github` → github.com/federicohyo
- **Branch**: `main`
- **Commit style**: imperative, short summary (e.g. "Fix VTEI intensity channel...")

## Known Issues

### Runtime crash: "terminate called without an active exception"
- **Symptom**: App aborts ~3 seconds after startup (around hot-pixel calibration time) with `std::terminate()`
- **Not a shutdown crash** — the `[ofxDVS] exit` log messages never appear, so `exit()` isn't reached
- **Partial mitigations already in place**:
  - `InferenceWorker::loop_()` now has try/catch around `job()` (commit c59fe87)
  - `ofxDVS::exit()` stops both workers, detaches USB thread as fallback
  - `ofApp::exit()` calls `dvs.exit()` during OF lifecycle
- **Investigation leads**:
  - The crash signature (`std::terminate` without active exception) means a joinable `std::thread` is being destroyed, OR something calls `std::terminate()` directly
  - Could be ONNX Runtime internal thread management (not our `std::thread` objects)
  - Could be `dv::io::camera` / dv-processing library spawning internal threads that crash
  - The `usbThread::threadedFunction()` uses `goto`-based state machine with `lock()`/`unlock()` on `ofThread`'s mutex — potential for deadlock or exception during locked state
  - `ofThread::run()` calls `thread.detach()` after `threadedFunction()` returns — if the thread crashes before that, the `std::thread` is still joinable
- **Next steps to try**:
  - Run with `gdb` or `valgrind` to get a stack trace at the point of `std::terminate()`
  - Add `std::set_terminate()` handler to print a backtrace before aborting
  - Check if the crash happens without the camera connected (file-only mode)
  - Check if disabling NN pipelines (don't call `yolo_worker.start()`) avoids the crash

## Common Tasks

- **Add a GUI control**: Add widget in `setup()` after line ~197, add handler in `onSliderEvent()`/`onButtonEvent()`/`onToggleEvent()`
- **Add a new filter**: Add members to private section of `ofxDVS.hpp` (~line 735), implement in `.cpp`, call from `update()` between `updateBAFilter()` and `updateImageGenerator()`
- **Add a new NN pipeline**: Create `dvs_*_pipeline.hpp/.cpp`, add `InferenceWorker<ResultT>` member, wire up in `update()` and `draw()`
