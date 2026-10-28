"""Regenerate tiny ONNX contract fixtures using only Python's standard library.

These are test graphs, never robot policies. Fields follow the ONNX schema:
https://github.com/onnx/onnx/blob/main/onnx/onnx.proto
The checked-in binaries let C++ tests run without Python/onnx/protobuf packages.
"""
from pathlib import Path
import struct


def varint(value):
    result = bytearray()
    while value > 127:
        result.append((value & 127) | 128)
        value >>= 7
    result.append(value)
    return bytes(result)


def integer(tag, value):
    return varint(tag << 3) + varint(value)


def field(tag, value):
    if isinstance(value, str):
        value = value.encode()
    return varint((tag << 3) | 2) + varint(len(value)) + value


def port(name, shape, dtype=1):
    dims = b"".join(field(1, field(2, dim) if isinstance(dim, str)
                          else integer(1, dim)) for dim in shape)
    tensor_type = integer(1, dtype) + field(2, dims)
    return field(1, name) + field(2, field(1, tensor_type))


def tensor(name, shape, values, dtype=7):
    raw = struct.pack("<" + ("q" if dtype == 7 else "f") * len(values), *values)
    return (b"".join(integer(1, dim) for dim in shape) + integer(2, dtype)
            + field(8, name) + field(9, raw))


def attribute(name, value):
    return field(1, name) + integer(3, value) + integer(20, 2)


def node(op, inputs, output, attrs=()):
    return (b"".join(field(1, name) for name in inputs) + field(2, output)
            + field(4, op) + b"".join(field(5, attr) for attr in attrs))


def write(name, inputs, outputs, nodes, tensors=()):
    graph = (field(2, name) + b"".join(field(1, n) for n in nodes)
             + b"".join(field(5, t) for t in tensors)
             + b"".join(field(11, p) for p in inputs)
             + b"".join(field(12, p) for p in outputs))
    model = integer(1, 8) + field(7, graph) + field(8, integer(2, 13))
    (ROOT / (name + ".onnx")).write_bytes(model)


ROOT = Path(__file__).with_name("fixtures")
ROOT.mkdir(exist_ok=True)
for name, width, batch, dtype, end, rank in [
    ("slice675", 675, 1, 1, 12, 2),
    ("slice705", 705, 1, 1, 12, 2),
    ("dynamic_batch", 675, "batch", 1, 12, 2),
    ("wrong_input_size", 676, 1, 1, 12, 2),
    ("wrong_output_size", 675, 1, 1, 13, 2),
    ("wrong_input_type", 675, 1, 11, 12, 2),
    ("wrong_rank", 675, 1, 1, 12, 3),
]:
    prefix = [batch] if rank == 2 else [batch, 1]
    write(name, [port("observations", prefix + [width], dtype)],
          [port("actions", prefix + [end], dtype)],
          [node("Slice", ["observations", "start", "end", "axis"], "actions")],
          [tensor("start", [1], [0]), tensor("end", [1], [end]),
           tensor("axis", [1], [rank - 1])])

inputs = [port("observations", [1, 675])]
outputs = [port("actions", [1, 12])]
slice_nodes = [node("Slice", ["observations", "start", "end", "axis"], "slice")]
slice_tensors = [tensor("start", [1], [0]), tensor("end", [1], [12]),
                 tensor("axis", [1], [1])]
write("wrong_output_type", inputs, [port("actions", [1, 12], 11)],
      slice_nodes + [node("Cast", ["slice"], "actions", [attribute("to", 11)])],
      slice_tensors)
write("two_inputs", inputs + [port("extra", [1, 675])], outputs,
      slice_nodes + [node("Identity", ["slice"], "actions")], slice_tensors)
write("two_outputs", inputs, outputs + [port("extra", [1, 12])],
      slice_nodes + [node("Identity", ["slice"], "actions"),
                     node("Identity", ["slice"], "extra")], slice_tensors)
write("warmup_nonfinite", inputs, outputs, [],
      [tensor("actions", [1, 12], [float("nan")] * 12, 1)])
write("output_overflow", inputs, outputs,
      slice_nodes + [node("Mul", ["slice", "large"], "actions")],
      slice_tensors + [tensor("large", [], [3e38], 1)])
# Zero input selects one row; observations[1] > 1 selects two rows.
# This exercises Run rejecting an output that exceeds the preallocated [1,12].
write("changing_output_batch", inputs, [port("actions", ["batch", 12])],
      [node("Slice", ["observations", "start", "end", "axis"], "pair"),
       node("Reshape", ["pair", "flat_shape"], "flat"),
       node("GreaterOrEqual", ["flat", "thresholds"], "mask"),
       node("Compress", ["rows", "mask"], "actions", [attribute("axis", 0)])],
      [tensor("start", [1], [0]), tensor("end", [1], [2]),
       tensor("axis", [1], [1]), tensor("flat_shape", [1], [2]),
       tensor("thresholds", [2], [0, 1], 1), tensor("rows", [2, 12], [0] * 24, 1)])
(ROOT / "corrupt.onnx").write_bytes(b"invalid ONNX model")
