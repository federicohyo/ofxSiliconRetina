#!/usr/bin/env python3
"""Add intermediate tensor outputs to ONNX model for activation probing."""
import onnx
from onnx import helper

src = "/home/federico/tue/of_v0.12.1_linux64_gcc6_release/addons/ofxDVS/example_dvs/bin/data/ReYOLOv8m_PEDRO_352x288.onnx"
dst = "/home/federico/tue/of_v0.12.1_linux64_gcc6_release/addons/ofxDVS/example_dvs/bin/data/ReYOLOv8m_PEDRO_352x288_probed.onnx"

# 12 probes in 3 groups of 4:
#   Group 0: LSTM hidden states h_t (gate Concat outputs, fixed batch=1)
#   Group 1: LSTM cell candidates g_t = Tanh_1 (dynamic batch=0 → resolved at run)
#   Group 2: Feedforward conv features (stem, SPPF, neck)
PROBES = [
    # Group 0 — LSTM hidden state
    "/model/backbone/stage1_gate/Concat_output_0",  # [1,128,72,88]
    "/model/backbone/stage2_gate/Concat_output_0",  # [1,256,36,44]
    "/model/backbone/stage3_gate/Concat_output_0",  # [1,512,18,22]
    "/model/backbone/stage4_gate/Concat_output_0",  # [1,1024,9,11]
    # Group 1 — LSTM cell state c_t = f*c_prev + i*g (accumulated temporal memory)
    "/model/backbone/stage1_gate/Add_1_output_0",   # [unk,64,72,88]
    "/model/backbone/stage2_gate/Add_1_output_0",   # [unk,128,36,44]
    "/model/backbone/stage3_gate/Add_1_output_0",   # [unk,256,18,22]
    "/model/backbone/stage4_gate/Add_1_output_0",   # [unk,512,9,11]
    # Group 2 — feedforward conv features
    "/model/backbone/stem2/act/Mul_output_0",              # [0,64,72,88]
    "/model/backbone/sppf/cv2/act/Mul_output_0",           # [0,512,9,11]
    "/model/neck/up_c2f_p3/cv1/act/Mul_output_0",          # [0,128,36,44]
    "/model/neck/down_c2f_p5/cv1/act/Mul_output_0",        # [0,512,9,11]
]

model = onnx.load(src)
print("Running shape inference...")
model = onnx.shape_inference.infer_shapes(model)
print("Done.")

graph = model.graph

type_map = {}
for vi in list(graph.value_info) + list(graph.output):
    type_map[vi.name] = vi.type

# Remove any stale probe outputs
for name in PROBES:
    stale = [o for o in graph.output if o.name == name]
    for o in stale:
        graph.output.remove(o)

for name in PROBES:
    if name in type_map:
        # Clone the ValueInfoProto directly so dynamic dims (dim_param) are preserved.
        # Using make_tensor_value_info with dim_value converts dynamic dims to 0,
        # which ORT treats as a static zero-size dimension and fails to run.
        vi_src = None
        for candidate in list(graph.value_info) + list(graph.output):
            if candidate.name == name:
                vi_src = candidate
                break
        if vi_src is not None:
            vi = onnx.ValueInfoProto()
            vi.CopyFrom(vi_src)
            vi.name = name
        else:
            # Fallback: build from type_map (may lose dynamic dim semantics)
            t = type_map[name].tensor_type
            shape_str = [f"d{i}:{d.dim_param or d.dim_value}" for i, d in enumerate(t.shape.dim)]
            vi = onnx.ValueInfoProto()
            vi.CopyFrom(onnx.helper.make_tensor_value_info(name, t.elem_type, None))
            vi.type.CopyFrom(type_map[name])
            vi.name = name
        shape_str = []
        for d in vi.type.tensor_type.shape.dim:
            shape_str.append(d.dim_param if d.dim_param else str(d.dim_value))
        print(f"  + {name}: [{', '.join(shape_str)}]")
    else:
        vi = onnx.ValueInfoProto()
        vi.name = name
        print(f"  + {name}: (no shape)")
    graph.output.append(vi)

print(f"\nFinal outputs ({len(graph.output)}):")
for o in graph.output:
    t = o.type.tensor_type
    shape = [d.dim_value for d in t.shape.dim] if t.HasField('shape') else "?"
    print(f"  {o.name}: shape={shape}")

onnx.save(model, dst)
print(f"\nSaved -> {dst}")
