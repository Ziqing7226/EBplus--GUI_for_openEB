#!/usr/bin/env python3
"""models/convert_hypere2vid_to_onnx.py — 将 HyperE2VID 的 .pth 模型转换为 ONNX 格式。

用法:
    .venv/bin/python models/convert_hypere2vid_to_onnx.py \
        --input /path/to/model.pth \
        --output models/hypere2vid.onnx

参考: ref/HyperE2VID/model/{model.py, unet.py, submodules.py, hyper/}（MIT License,
IEEE TIP 2024 "HyperE2VID: Improving Event-Based Video Reconstruction via Hypernetworks"）。

checkpoint 是 ACDA 训练框架的完整训练存档（arch / state_dict / config 三键，config
被 pickle 为 parse_config.ConfigParser 对象）。训练仓库无需克隆：加载前向 sys.modules
注册一个 parse_config stub 即可，模型构造参数直接取自 checkpoint 内嵌的 config。

导出接口与 C++ 端 e2vid_inference.h 的 N 输入 / M 输出循环处理按位置对齐:
    inputs : [event_tensor, h0, c0, h1, c1, h2, c2, prev_recs]   (8)
    outputs: [image,         h0', c0', h1', c1', h2', c2', image] (8)
即 output[i+1] == 下一帧 input[i+1]（ConvLSTM 状态 + prev_recs == image 自反馈）。

依赖: torch (CPU)、onnx、numpy、scipy（HyperE2VID 的 Fourier-Bessel 基构造需要）。
"""

import argparse
import os
import sys

import numpy as np
import torch  # 必须先于 parse_config stub 导入（stub 会干扰 inspect 对 sys.modules 的遍历）
import onnx
import onnx.checker


# ---------------------------------------------------------------------------
# checkpoint 加载（ACDA 训练框架存档，含 pickled ConfigParser）
# ---------------------------------------------------------------------------

def _install_parse_config_stub():
    """注册 parse_config stub，使 torch.load 能反序列化 checkpoint 里的 ConfigParser。"""
    import types

    mod = types.ModuleType("parse_config")

    class _StubObject:
        def __setstate__(self, state):
            if isinstance(state, dict):
                self.__dict__.update(state)
            else:
                self._state = state

    def _getattr(name):
        if name.startswith("__"):
            raise AttributeError(name)
        return type(name, (_StubObject,), {})

    mod.__getattr__ = _getattr
    sys.modules["parse_config"] = mod


def load_model(path_to_model, hypere2vid_dir):
    """加载 HyperE2VID checkpoint 并实例化模型（arch 恒为 E2VIDRecurrent）。"""
    sys.path.insert(0, hypere2vid_dir)

    import model.submodules as _submodules  # noqa: E402

    if not hasattr(_submodules, "TransposedConvLayer"):
        # 上游 unet.py 顶层 import 了 submodules.py 未定义的 TransposedConvLayer
        # （发布版仓库的潜在 bug）。预训练配置 use_upsample_conv=True 用不到它，
        # 补一个无害别名让 import 通过，不改动 ref/ 上游代码。
        _submodules.TransposedConvLayer = _submodules.UpsampleConvLayer

    from model.model import E2VIDRecurrent  # noqa: E402

    _install_parse_config_stub()
    print(f"Loading model {path_to_model} ...")
    raw_model = torch.load(path_to_model, map_location="cpu", weights_only=False)

    arch = raw_model["arch"]
    if arch != "E2VIDRecurrent":
        raise ValueError(f"unsupported arch '{arch}'")

    config = raw_model["config"]._config
    unet_kwargs = config["arch"]["args"]["unet_kwargs"]
    print(f"  arch={arch}, unet_kwargs={unet_kwargs}")

    model = E2VIDRecurrent(unet_kwargs)
    model.load_state_dict(raw_model["state_dict"])  # strict: bases buffer 含于 checkpoint
    model.eval()
    return model


# ---------------------------------------------------------------------------
# ONNX 导出包装器
# ---------------------------------------------------------------------------

class HyperE2VIDONNXWrapper(torch.nn.Module):
    """显式前向展开，把全部循环状态（ConvLSTM h/c × 3 + prev_recs）提升为图输入/输出。

    不能直接包装 E2VIDRecurrent.forward：其 states / prev_recs 持有在模块属性里，
    追踪导出时非参数张量属性会被固化成常量（首帧零状态被烘焙进图，C++ 端喂入的
    真实状态被忽略）。因此这里按 UNetRecurrent.forward 的顺序手动走一遍子模块，
    状态全部经函数参数传递。
    """

    def __init__(self, model):
        super().__init__()
        self.model = model
        from model.submodules import DynamicUpsampleLayer  # noqa: E402
        from model.model_util import skip_sum  # noqa: E402

        self._dynamic_layer_cls = DynamicUpsampleLayer
        self._skip_sum = skip_sum

    def forward(self, event_tensor, h0, c0, h1, c1, h2, c2, prev_recs):
        u = self.model.unetrecurrent
        skip_sum = self._skip_sum
        states = [(h0, c0), (h1, c1), (h2, c2)]

        ev_tensor = event_tensor
        x = u.head(event_tensor)
        head = x

        blocks = []
        for i, encoder in enumerate(u.encoders):
            x, state = encoder(x, states[i])
            blocks.append(x)
            states[i] = state

        for resblock in u.resblocks:
            x = resblock(x)

        for i, decoder in enumerate(u.decoders):
            skip_from_encoder = blocks[len(states) - i - 1]
            if isinstance(decoder, self._dynamic_layer_cls):
                x = decoder(skip_sum(x, skip_from_encoder), ev_tensor, prev_recs)
            else:
                x = decoder(skip_sum(x, skip_from_encoder))

        image = u.pred(skip_sum(x, head))

        new_states = []
        for hidden, cell in states:
            new_states.append(hidden)
            new_states.append(cell)

        # 末位输出 = 下一帧的 prev_recs 输入，恰好等于本帧 image（按位置反馈）。
        return (image, *new_states, image)


# ---------------------------------------------------------------------------
# 导出
# ---------------------------------------------------------------------------

def export(model, output_path, opset=17):
    """导出 HyperE2VID（8 入 8 出，prev_recs 位于输入/输出末位）。"""
    num_encoders = model.num_encoders
    num_bins = model.num_bins
    base_ch = model.unetrecurrent.base_num_channels
    if num_encoders != 3:
        raise ValueError(
            f"本脚本目前仅支持 num_encoders=3 的模型（当前={num_encoders}）。"
            "如需其他配置，请按相同模式增减 h/c 参数。"
        )

    wrapper = HyperE2VIDONNXWrapper(model)

    # 256x256 可被 2^3=8 整除（context fusion 再 /4 也整除）。
    dummy_h, dummy_w = 256, 256
    dummy_event = torch.zeros(1, num_bins, dummy_h, dummy_w)
    dummy_prev = torch.zeros(1, 1, dummy_h, dummy_w)

    ch_levels = [base_ch * (2 ** (i + 1)) for i in range(num_encoders)]
    sp_levels = [dummy_h // (2 ** (i + 1)) for i in range(num_encoders)]

    dummy_states = []
    for ch, sp in zip(ch_levels, sp_levels):
        dummy_states.append(torch.zeros(1, ch, sp, sp))  # h
        dummy_states.append(torch.zeros(1, ch, sp, sp))  # c

    input_names = ["event_tensor", "h0", "c0", "h1", "c1", "h2", "c2", "prev_recs"]
    output_names = ["image", "h0_new", "c0_new", "h1_new", "c1_new", "h2_new", "c2_new", "prev_recs_new"]

    dynamic_axes = {
        "event_tensor": {0: "batch", 2: "H", 3: "W"},
        "prev_recs": {0: "batch", 2: "H", 3: "W"},
        "prev_recs_new": {0: "batch", 2: "H", 3: "W"},
        "image": {0: "batch", 2: "H", 3: "W"},
    }
    level_dim_names = [("H2", "W2"), ("H4", "W4"), ("H8", "W8")]
    for i in range(num_encoders):
        hn, cn = f"h{i}", f"c{i}"
        hn_new, cn_new = f"h{i}_new", f"c{i}_new"
        hd, wd = level_dim_names[i]
        for name in (hn, cn, hn_new, cn_new):
            dynamic_axes[name] = {0: "batch", 2: hd, 3: wd}

    print(f"Exporting to {output_path} ...")
    torch.onnx.export(
        wrapper,
        (dummy_event, *dummy_states, dummy_prev),
        output_path,
        input_names=input_names,
        output_names=output_names,
        dynamic_axes=dynamic_axes,
        opset_version=opset,
        do_constant_folding=True,
        dynamo=False,  # 使用传统导出器（兼容性更好）
    )
    print("  Export complete.")


def verify(output_path):
    """验证导出的 ONNX 模型。"""
    print(f"Verifying {output_path} ...")
    onnx_model = onnx.load(output_path)
    onnx.checker.check_model(onnx_model)

    for i, inp in enumerate(onnx_model.graph.input):
        shape = [d.dim_value or d.dim_param or "?" for d in inp.type.tensor_type.shape.dim]
        print(f"  input[{i}]  {inp.name}: {shape}")
    for i, out in enumerate(onnx_model.graph.output):
        shape = [d.dim_value or d.dim_param or "?" for d in out.type.tensor_type.shape.dim]
        print(f"  output[{i}] {out.name}: {shape}")
    print("  ONNX model is valid.")


def main():
    parser = argparse.ArgumentParser(description="Convert HyperE2VID .pth to ONNX")
    parser.add_argument("--input", required=True, help="Path to model.pth")
    parser.add_argument("--output", required=True, help="Path to output .onnx model")
    parser.add_argument(
        "--hypere2vid-dir",
        default=None,
        help="Path to ref/HyperE2VID (default: auto-detect <repo>/ref/HyperE2VID)",
    )
    parser.add_argument("--opset", type=int, default=17, help="ONNX opset version")
    args = parser.parse_args()

    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    he2v_dir = args.hypere2vid_dir or os.path.join(repo_root, "ref", "HyperE2VID")
    if not os.path.isdir(he2v_dir):
        print(f"Error: {he2v_dir} not found. Clone HyperE2VID first or use --hypere2vid-dir.")
        sys.exit(1)

    model = load_model(args.input, he2v_dir)
    export(model, args.output, opset=args.opset)
    verify(args.output)


if __name__ == "__main__":
    main()
