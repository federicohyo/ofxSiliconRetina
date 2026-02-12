# ofxDVS

An [openFrameworks](https://openframeworks.cc/) addon for interfacing Dynamic Vision Sensor (DVS / event) cameras with built-in neural-network inference pipelines for real-time object detection and gesture recognition.

## Features

- **Camera I/O** via [dv-processing](https://gitlab.com/inivation/dv/dv-processing) (auto-detects connected camera)
- **YOLO object detection** on a 5-channel VTEI (Visual-Temporal Event Image) representation, with asynchronous inference on a worker thread
- **TSDT gesture recognition** using temporal-spatial binary event tensors with EMA-smoothed logits
- **Rectangular Cluster Tracker** for event-driven multi-object tracking
- **ONNX Runtime** inference backend with multi-threaded execution and float16 support
- 2D and 3D event visualization, APS frame display, IMU overlay
- AEDAT 3.1 file recording and playback
- Hot-pixel suppression via configurable refractory period filter
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
| ofxGui | OF addon (core) | Additional GUI elements |
| ofxPoco | OF addon (core) | Networking utilities |
| ofxNetwork | OF addon (core) | Network I/O |

## Installation

1. Clone or copy this addon into `openFrameworks/addons/ofxDVS/`.
2. Install [dv-processing](https://gitlab.com/inivation/dv/dv-processing) system-wide (follow their build instructions).
3. Install [ofxDatGui](https://github.com/braitsch/ofxDatGui) into `openFrameworks/addons/`.
4. The bundled `libcaer` and `onnxruntime` libraries are in `libs/`. If they don't match your architecture, rebuild and copy them there.

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

## Hotkeys

| Key | Action |
|-----|--------|
| `c` | Toggle GUI visibility |

GUI toggles and sliders control all other functionality (DVS, APS, IMU, tracker, NN, recording, etc.) via the panel interface.

## ONNX Models

Place model files in `example_dvs/bin/data/`. The code loads:

- **YOLO**: `ReYOLOv8m_PEDRO_352x288.onnx` (object detection)
- **TSDT**: `spikevision_822128128_fixed.onnx` (gesture recognition)

## License

[MIT License](license.md) &mdash; Copyright (c) 2017 Federico Corradi
