# example_dvs

Minimal example application for the ofxDVS addon. Connects to a DVS camera, displays events, and runs neural-network inference pipelines.

## Running

```bash
cd example_dvs
make && make run
```

The application opens a 1024x768 window and automatically connects to the first available DVS camera via `dv::io::camera::open()`.

## ONNX Model Files

Place the following models in `bin/data/`:

| File | Pipeline | Notes |
|------|----------|-------|
| `ReYOLOv8m_PEDRO_352x288.onnx` | YOLO object detection | 352x288 input, 1 class ("person") |
| `spikevision_822128128_fixed.onnx` | TSDT gesture recognition | T=8, 2x128x128 input, 11 gesture classes |

Both models are loaded at startup. If a model file is missing, the corresponding pipeline is disabled with a log warning.

## GUI Controls

Press **`c`** to toggle the main GUI panel. The panel provides:

- **DVS / APS / IMU** toggles for data streams
- **DVS Image Gen** toggle and accumulation slider
- **ENABLE NEURAL NETS** toggle (shows/hides the NN sub-panel)
- **ENABLE TRACKER** toggle (shows/hides the tracker sub-panel)
- **Recording** start/stop and file loading
- **Refractory (us)** slider for hot-pixel filtering

### Neural Net Panel (when enabled)

**YOLO folder:**
- ENABLE NN, DRAW DETECTIONS, SHOW LABELS toggles
- CONF THRESH, IOU THRESH, SMOOTH FRAMES, VTEI Window (ms) sliders
- CLEAR HISTORY button

**TSDT folder:**
- ENABLE TSDT, SHOW LABEL toggles
- TIMESTEPS (T), BIN (ms), EMA alpha sliders
- SELFTEST (from file) button

### Tracker Panel (when enabled)

Full control over the Rectangular Cluster Tracker parameters: cluster count, size, velocity, paths, merging, smoothing, and more.

## Camera Troubleshooting

- Ensure the camera is connected via USB and recognized by the system (`lsusb`)
- The dv-processing library must be installed system-wide
- Set the correct `#define` in `src/ofxDVS.hpp` (`DAVIS`, `DVS128`, or `DVXPLORER`) to match your camera family
- If no camera is found, the application will log an error and fall back to an unconnected state
- On Linux, you may need udev rules for iniVation cameras (see dv-processing documentation)
