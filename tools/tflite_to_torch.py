"""Rebuild an int8 TFLite graph as an equivalent float PyTorch module.

Why this exists
---------------
The Sony IMX500 toolchain (`imxconv-pt` / `imxconv-tf`) does NOT accept a
`.tflite` file. It takes a model that MCT — Sony's Model Compression Toolkit —
has quantized against the IMX500 target platform, exported as ONNX (PyTorch
path) or Keras (TensorFlow path). MCT in turn wants a FLOAT model to quantize.

So an already-int8 Edge Impulse `.lite` export has to be walked backwards:

    .lite (int8)  ->  float PyTorch  ->  MCT PTQ (IMX500 TPC)  ->  ONNX  ->  .rpk
    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    this file

The usual way to get that first arrow is tf2onnx + onnx2torch, which drags in a
pinned TensorFlow and a NHWC->NCHW transpose soup that MCT then has to fold. The
FOMO graph Edge Impulse emits is small and boring enough — Conv2D,
DepthwiseConv2D, Add, Softmax, nothing else — that reading the flatbuffer
directly and emitting real `nn.Conv2d`s is both shorter and produces a cleaner
graph for the converter. It also means no TensorFlow anywhere in the pipeline.

The float weights recovered here are EXACT: int8 TFLite stores per-channel
scales, so dequantization is lossless with respect to what the .lite file
actually computes. What is NOT recovered is the pre-quantization original — the
rounding Edge Impulse already baked in is permanent, and re-quantizing for the
IMX500 stacks a second rounding on top of it. See MODEL_CONVERSION.md.

Layout note: TFLite is NHWC, PyTorch is NCHW. Weights are transposed at build
time so no runtime transposes survive into the graph. The module therefore takes
NCHW input; `TFLiteModel.output_is_nhwc` controls whether a trailing permute
restores the original NHWC output ordering.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

import tflite
from tflite import ActivationFunctionType, BuiltinOperator, Padding, TensorType


# --- flatbuffer reading ------------------------------------------------------

def _builtin_code(model, op) -> int:
    """Opcode, tolerating the deprecated-vs-extended split in the schema.

    Codes below 127 historically lived in `deprecated_builtin_code` (an int8
    field) and newer ones in `builtin_code`. Readers are expected to prefer the
    larger of the two, which is exactly what the schema comment says to do.
    """
    oc = model.OperatorCodes(op.OpcodeIndex())
    code = oc.BuiltinCode()
    try:
        dep = oc.DeprecatedBuiltinCode()
    except Exception:
        dep = 0
    return max(code, dep)


def _tensor_array(model, subgraph, index: int) -> Optional[np.ndarray]:
    """Raw (still-quantized) contents of a constant tensor, or None if dynamic."""
    t = subgraph.Tensors(index)
    buf = model.Buffers(t.Buffer())
    if buf.DataLength() == 0:
        return None
    raw = buf.DataAsNumpy().tobytes()
    dtype = {
        TensorType.INT8: np.int8,
        TensorType.UINT8: np.uint8,
        TensorType.INT16: np.int16,
        TensorType.INT32: np.int32,
        TensorType.INT64: np.int64,
        TensorType.FLOAT32: np.float32,
    }[t.Type()]
    return np.frombuffer(raw, dtype=dtype).reshape(t.ShapeAsNumpy().tolist())


def _quant(subgraph, index: int) -> Tuple[np.ndarray, np.ndarray, int]:
    """(scales, zero_points, quantized_dimension) for a tensor."""
    q = subgraph.Tensors(index).Quantization()
    if q is None or q.ScaleLength() == 0:
        return np.array([1.0], np.float64), np.array([0], np.int64), 0
    return (q.ScaleAsNumpy().astype(np.float64),
            q.ZeroPointAsNumpy().astype(np.int64),
            q.QuantizedDimension())


def _dequantize(model, subgraph, index: int) -> np.ndarray:
    """Constant tensor as float64, undoing per-tensor or per-channel int8 scaling."""
    raw = _tensor_array(model, subgraph, index)
    if raw is None:
        raise ValueError(f"tensor {index} has no constant buffer")
    if raw.dtype == np.float32:
        return raw.astype(np.float64)
    scales, zeros, qdim = _quant(subgraph, index)
    # Broadcast the per-channel axis against the tensor's rank: a length-N scale
    # vector on axis `qdim` becomes shape (1,..,N,..,1).
    shape = [1] * raw.ndim
    if scales.size > 1:
        shape[qdim] = scales.size
    return (raw.astype(np.float64) - zeros.reshape(shape)) * scales.reshape(shape)


# --- padding -----------------------------------------------------------------

def _same_padding(in_size: int, k: int, stride: int, dilation: int) -> Tuple[int, int]:
    """TensorFlow's SAME padding, which is allowed to be ASYMMETRIC.

    For a 240x240 input with a 3x3 stride-2 kernel TF pads (0, 1) — nothing on
    top/left, one row on bottom/right. `nn.Conv2d`'s `padding=` is symmetric
    only, so anything asymmetric has to become an explicit pad before the conv,
    or the whole feature map shifts half a pixel and every downstream activation
    is subtly wrong.
    """
    eff_k = (k - 1) * dilation + 1
    out = math.ceil(in_size / stride)
    total = max((out - 1) * stride + eff_k - in_size, 0)
    before = total // 2
    return before, total - before


# --- the module ---------------------------------------------------------------

@dataclass
class _Step:
    """One node of the replayed graph. Plain ints/strings so fx can trace it."""
    kind: str                  # "conv" | "add" | "softmax"
    out: int                   # output tensor index
    ins: Tuple[int, ...]       # input tensor indices
    module: int = -1           # index into TFLiteModel.ops, -1 if none
    act: str = "none"          # fused activation: none | relu | relu6 | relu_n1_1


_ACTIVATIONS = {
    ActivationFunctionType.NONE: "none",
    ActivationFunctionType.RELU: "relu",
    ActivationFunctionType.RELU6: "relu6",
    ActivationFunctionType.RELU_N1_TO_1: "relu_n1_1",
}


class TFLiteModel(nn.Module):
    """Replays a parsed TFLite subgraph with native PyTorch ops.

    `forward` walks a STATIC plan built at load time — no data-dependent control
    flow — so `torch.fx.symbolic_trace` (which MCT uses to read the model)
    unrolls it into a flat graph of Conv/ReLU/Add/Softmax with nothing left of
    the interpreter loop.
    """

    def __init__(self, steps: Sequence[_Step], ops: Sequence[nn.Module],
                 input_index: int, output_index: int, output_is_nhwc: bool):
        super().__init__()
        self.ops = nn.ModuleList(ops)
        self._steps = list(steps)
        self._input_index = input_index
        self._output_index = output_index
        self.output_is_nhwc = output_is_nhwc

    def _activate(self, x, act: str):
        if act == "relu":
            return F.relu(x)
        if act == "relu6":
            return F.hardtanh(x, 0.0, 6.0)
        if act == "relu_n1_1":
            return F.hardtanh(x, -1.0, 1.0)
        return x

    def forward(self, x):
        vals: Dict[int, torch.Tensor] = {self._input_index: x}
        for s in self._steps:
            if s.kind == "conv":
                y = self.ops[s.module](vals[s.ins[0]])
            elif s.kind == "add":
                y = vals[s.ins[0]] + vals[s.ins[1]]
            elif s.kind == "softmax":
                # TFLite softmaxes the last (channel) axis of NHWC; in NCHW
                # that is axis 1.
                y = F.softmax(vals[s.ins[0]], dim=1)
            else:  # pragma: no cover - guarded at build time
                raise RuntimeError(f"unhandled step {s.kind}")
            vals[s.out] = self._activate(y, s.act)
        out = vals[self._output_index]
        if self.output_is_nhwc:
            out = out.permute(0, 2, 3, 1)
        return out


@dataclass
class ModelMeta:
    """What the caller needs to drive the model that the graph itself doesn't say."""
    input_shape_nchw: Tuple[int, int, int, int]
    input_scale: float
    input_zero_point: int
    output_scale: float
    output_zero_point: int
    num_classes: int
    grid: Tuple[int, int]
    is_float: bool = False
    op_histogram: Dict[str, int] = field(default_factory=dict)

    @property
    def input_range(self) -> Tuple[float, float]:
        """Float range the model's input spans — the preprocessing contract.

        An int8 export states it outright: Edge Impulse emits scale=1/255,
        zp=-128, so the int8 codes [-128, 127] map to exactly [0, 1]. A float32
        export states NOTHING — the graph would happily accept [0, 255] — so the
        contract has to come from elsewhere. It is assumed to be [0, 1], which is
        Edge Impulse's convention and is cross-checked against the int8 sibling
        export in tests/test_tflite_to_torch.py: feeding both models the same
        [0, 1] image produces the same detections, which it would not if the
        float model wanted a different range.
        """
        if self.is_float:
            return 0.0, 1.0
        lo = (-128 - self.input_zero_point) * self.input_scale
        hi = (127 - self.input_zero_point) * self.input_scale
        return lo, hi


def load(path: str, output_is_nhwc: bool = False) -> Tuple[TFLiteModel, ModelMeta]:
    """Parse `path` and return an equivalent float PyTorch module plus metadata.

    Raises on any operator outside the supported set rather than silently
    emitting a model that computes something else — a wrong-but-runnable network
    is far worse here than a failed conversion, because nothing downstream would
    catch it.
    """
    with open(path, "rb") as fh:
        buf = bytearray(fh.read())
    model = tflite.Model.GetRootAsModel(buf, 0)
    if model.SubgraphsLength() != 1:
        raise ValueError(f"expected 1 subgraph, got {model.SubgraphsLength()}")
    sg = model.Subgraphs(0)

    if sg.InputsLength() != 1 or sg.OutputsLength() != 1:
        raise ValueError("expected a single-input, single-output graph")
    in_idx = int(sg.Inputs(0))
    out_idx = int(sg.Outputs(0))

    in_shape = sg.Tensors(in_idx).ShapeAsNumpy().tolist()   # NHWC
    if len(in_shape) != 4 or in_shape[3] not in (1, 3):
        raise ValueError(f"expected NHWC image input, got {in_shape}")

    steps: List[_Step] = []
    ops: List[nn.Module] = []
    histogram: Dict[str, int] = {}
    # Spatial size of each tensor, needed to compute SAME padding. Seeded with
    # the graph input and extended as we walk forward.
    hw: Dict[int, Tuple[int, int]] = {in_idx: (in_shape[1], in_shape[2])}

    for i in range(sg.OperatorsLength()):
        op = sg.Operators(i)
        code = _builtin_code(model, op)
        name = {
            BuiltinOperator.CONV_2D: "CONV_2D",
            BuiltinOperator.DEPTHWISE_CONV_2D: "DEPTHWISE_CONV_2D",
            BuiltinOperator.ADD: "ADD",
            BuiltinOperator.SOFTMAX: "SOFTMAX",
        }.get(code)
        if name is None:
            raise ValueError(
                f"operator #{i} (builtin code {code}) is not supported by this "
                "reconstructor. It was written for the Conv/DepthwiseConv/Add/"
                "Softmax subset an Edge Impulse FOMO export uses.")
        histogram[name] = histogram.get(name, 0) + 1

        o_idx = int(op.Outputs(0))
        o_shape = sg.Tensors(o_idx).ShapeAsNumpy().tolist()

        if name in ("CONV_2D", "DEPTHWISE_CONV_2D"):
            x_idx, w_idx = int(op.Inputs(0)), int(op.Inputs(1))
            b_idx = int(op.Inputs(2)) if op.InputsLength() > 2 else -1
            depthwise = name == "DEPTHWISE_CONV_2D"

            if depthwise:
                o = tflite.DepthwiseConv2DOptions()
            else:
                o = tflite.Conv2DOptions()
            o.Init(op.BuiltinOptions().Bytes, op.BuiltinOptions().Pos)
            stride = (o.StrideH(), o.StrideW())
            dilation = (o.DilationHFactor(), o.DilationWFactor())
            act = _ACTIVATIONS[o.FusedActivationFunction()]

            w = _dequantize(model, sg, w_idx)
            if depthwise:
                # TFLite [1, kh, kw, in_ch*mult] -> torch [in_ch*mult, 1, kh, kw]
                mult = o.DepthMultiplier()
                kh, kw = w.shape[1], w.shape[2]
                groups = w.shape[3] // mult
                weight = w.reshape(kh, kw, w.shape[3]).transpose(2, 0, 1)[:, None]
                in_ch, out_ch = groups, w.shape[3]
            else:
                # TFLite [out_ch, kh, kw, in_ch] -> torch [out_ch, in_ch, kh, kw]
                weight = w.transpose(0, 3, 1, 2)
                kh, kw = w.shape[1], w.shape[2]
                out_ch, in_ch, groups = w.shape[0], w.shape[3], 1

            ih, iw = hw[x_idx]
            if o.Padding() == Padding.SAME:
                ph = _same_padding(ih, kh, stride[0], dilation[0])
                pw = _same_padding(iw, kw, stride[1], dilation[1])
            else:
                ph = pw = (0, 0)

            conv = nn.Conv2d(in_ch, out_ch, (kh, kw), stride=stride,
                             padding=0, dilation=dilation, groups=groups,
                             bias=b_idx >= 0)
            conv.weight.data = torch.from_numpy(np.ascontiguousarray(weight)).float()
            if b_idx >= 0:
                bias = _dequantize(model, sg, b_idx)
                conv.bias.data = torch.from_numpy(np.ascontiguousarray(bias)).float()

            symmetric = ph[0] == ph[1] and pw[0] == pw[1]
            if symmetric:
                conv.padding = (ph[0], pw[0])
                module: nn.Module = conv
            else:
                # F.pad order is (left, right, top, bottom).
                module = nn.Sequential(
                    nn.ZeroPad2d((pw[0], pw[1], ph[0], ph[1])), conv)

            steps.append(_Step("conv", o_idx, (x_idx,), len(ops), act))
            ops.append(module)

        elif name == "ADD":
            o = tflite.AddOptions()
            o.Init(op.BuiltinOptions().Bytes, op.BuiltinOptions().Pos)
            steps.append(_Step("add", o_idx,
                               (int(op.Inputs(0)), int(op.Inputs(1))),
                               -1, _ACTIVATIONS[o.FusedActivationFunction()]))

        else:  # SOFTMAX
            o = tflite.SoftmaxOptions()
            o.Init(op.BuiltinOptions().Bytes, op.BuiltinOptions().Pos)
            if abs(o.Beta() - 1.0) > 1e-6:
                raise ValueError(f"softmax beta={o.Beta()} is not supported")
            steps.append(_Step("softmax", o_idx, (int(op.Inputs(0)),)))

        hw[o_idx] = (o_shape[1], o_shape[2])

    in_scale, in_zp, _ = _quant(sg, in_idx)
    out_scale, out_zp, _ = _quant(sg, out_idx)
    out_shape = sg.Tensors(out_idx).ShapeAsNumpy().tolist()
    # A float32 export carries no quantization params at all, so `_quant`'s
    # (1.0, 0) fallback is what comes back — meaningless as a scale, and
    # ModelMeta must not present it as one.
    is_float = sg.Tensors(in_idx).Type() == TensorType.FLOAT32

    meta = ModelMeta(
        input_shape_nchw=(1, in_shape[3], in_shape[1], in_shape[2]),
        input_scale=float(in_scale[0]), input_zero_point=int(in_zp[0]),
        output_scale=float(out_scale[0]), output_zero_point=int(out_zp[0]),
        num_classes=int(out_shape[3]), grid=(out_shape[1], out_shape[2]),
        is_float=is_float, op_histogram=histogram,
    )
    net = TFLiteModel(steps, ops, in_idx, out_idx, output_is_nhwc).eval()
    return net, meta
