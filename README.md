# ofxDVS

An [openFrameworks](https://openframeworks.cc/) addon for interfacing Dynamic Vision Sensor (DVS / event) cameras with built-in neural-network inference pipelines for real-time object detection and gesture recognition.

## Features

- **Camera I/O** via [dv-processing](https://gitlab.com/inivation/dv/dv-processing) (auto-detects connected camera)
- **YOLO object detection** on a 5-channel VTEI (Visual-Temporal Event Image) representation, with asynchronous inference on a worker thread
- **TSDT gesture recognition** using temporal-spatial binary event tensors with EMA-smoothed logits
- **Rectangular Cluster Tracker** for event-driven multi-object tracking
- **ONNX Runtime** inference backend with multi-threaded execution and float16 support
- **Event reconstruction** with per-pixel exponential decay, spatial spread, and ON/OFF color coding (yellow/blue)
- **MP4 video recording** of the viewer output via [ofxFFmpegRecorder](https://github.com/nickhobbs94/ofxFFmpegRecorder) (pipes to system ffmpeg)
- 2D and 3D event visualization, APS frame display, IMU overlay
- AEDAT 3.1 and AEDAT4 file recording and playback with real-time speed control
- Hot-pixel suppression via startup calibration mask, refractory period, and rate-based filtering
- Full GUI controls via [ofxDatGui](https://github.com/braitsch/ofxDatGui)

## Supported Cameras

| Family | Models | Resolution |
|--------|--------|------------|
| DVS128 | DVS128 | 128 x 128 |
| DAVIS | DAVIS128, DAVIS208, DAVIS240A/B/C, DAVIS346A/B/Cbsi, DAVIS640, DAVISHet640 | varies |
| DVXplorer | DVXplorer | 640 x 480 |

Select the camera family by setting the `#define` flags in `src/ofxDVS.hpp` (`DAVIS`, `DVS128`, or `DVXPLORER`).

## Dependencies

### Bundled (in `libs/`)

- **libcaer** &mdash; C driver library for iniVation cameras
- **onnxruntime** &mdash; ONNX Runtime C++ inference engine

### External (system / OF addons)

| Dependency | Type | Notes |
|-----------|------|-------|
| [openFrameworks 0.12.0](https://openframeworks.cc/) | Framework | Linux 64-bit tested |
| [dv-processing](https://gitlab.com/inivation/dv/dv-processing) | System library | Camera I/O (`dv::io::camera`) |
| [ofxDatGui](https://github.com/braitsch/ofxDatGui) | OF addon | GUI panels |
| [ofxFFmpegRecorder](https://github.com/nickhobbs94/ofxFFmpegRecorder) | OF addon | MP4 video recording (requires system `ffmpeg`) |
| ofxGui | OF addon (core) | Additional GUI elements |
| ofxPoco | OF addon (core) | Networking utilities |
| ofxNetwork | OF addon (core) | Network I/O |

## Installation

1. Clone or copy this addon into `openFrameworks/addons/ofxDVS/`.
2. Install [dv-processing](https://gitlab.com/inivation/dv/dv-processing) system-wide (follow their build instructions).
3. Install [ofxDatGui](https://github.com/braitsch/ofxDatGui) into `openFrameworks/addons/`.
4. Install [ofxFFmpegRecorder](https://github.com/nickhobbs94/ofxFFmpegRecorder) into `openFrameworks/addons/`.
5. Make sure `ffmpeg` is installed system-wide (`sudo apt install ffmpeg` on Ubuntu).
6. The bundled `libcaer` and `onnxruntime` libraries are in `libs/`. If they don't match your architecture, rebuild and copy them there.

## Building the Example

```bash
cd example_dvs
make
make run
```

Or use the openFrameworks Project Generator to create an IDE project that includes `ofxDVS` and `ofxDatGui`.

## Architecture

```
src/
  ofxDVS.hpp / .cpp              Core addon class (camera I/O, event processing, draw, GUI)
  dvs_yolo_pipeline.hpp / .cpp   YOLO detection pipeline (VTEI build, inference, NMS, drawing)
  dvs_tsdt_pipeline.hpp / .cpp   TSDT gesture pipeline (event history, tensor build, inference)
  dvs_nn_utils.hpp               Shared NN utilities (sigmoid, NMS, letterbox transforms)
  dvs_gui.hpp / .cpp             NN and tracker GUI panel creation + event handlers
  dvs_inference_worker.hpp       Thread-safe async inference worker (template)
  onnx_run.hpp / .cpp            ONNX Runtime wrapper (load, run, FP16 support)
  RectangularClusterTracker.hpp / .cpp   Event-driven cluster tracker
  ofxDvsPolarity.hpp             Polarity event data structures
```

## Event Reconstruction

The addon includes real-time event-driven image reconstruction. Since DVS cameras only output per-pixel brightness *changes* (not absolute intensity), this feature integrates events over time to reconstruct a continuous image of the scene.

**How it works:**
- Each pixel maintains a signed intensity value (-1.0 to +1.0)
- Every frame, all pixels decay towards zero (exponential decay)
- Incoming ON events add intensity; OFF events subtract it
- Events spread to neighboring pixels within a configurable radius, weighted by distance
- Positive values render as **yellow** (ON activity), negative as **blue** (OFF activity), fading to black

**GUI controls:**
| Slider | Range | Description |
|--------|-------|-------------|
| Recon Decay | 0.90 &ndash; 1.0 | Per-frame decay factor. Lower = faster fade, higher = longer trails |
| Recon Contrib | 0.01 &ndash; 0.5 | Intensity added per event. Higher = brighter pops |
| Recon Spread | 1 &ndash; 8 | Spatial spread radius in pixels. Higher = softer bleed |

Toggle **Recon Image** in the GUI to enable/disable.

## Video Recording

The addon can record the viewer output to MP4 video using ffmpeg. Controls are in the **>> Video Output** folder on the NN panel.

| Button / Slider | Description |
|----------------|-------------|
| START / STOP RECORDING MP4 | Begin or end recording. Output: `bin/data/dvs_video_<timestamp>.mp4` |
| PAUSE / RESUME RECORDING | Pause recording without starting a new file |
| REC FPS | Frame rate for the output video (10 &ndash; 60, default 30) |

Workflow for stitching multiple AEDAT files into one video: start recording, load the first file, pause, load the next file, resume, and so on. Click STOP when done.

## Hotkeys

| Key | Action |
|-----|--------|
| `c` | Toggle GUI visibility |

GUI toggles and sliders control all other functionality (DVS, APS, IMU, tracker, NN, recording, etc.) via the panel interface.

## ONNX Models

Place model files in `example_dvs/bin/data/`. The code loads:

- **YOLO**: `ReYOLOv8m_PEDRO_352x288.onnx` (object detection)
- **TSDT**: `spikevision_822128128_fixed.onnx` (gesture recognition)
- **TPDVSGesture**: `tp_gesture_paper_32x32.onnx` (gesture classification)

## License

[MIT License](license.md) &mdash; Copyright (c) 2017 Federico Corradi
