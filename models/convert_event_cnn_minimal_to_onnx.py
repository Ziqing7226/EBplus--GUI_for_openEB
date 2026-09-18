#!/usr/bin/env python3
"""models/convert_event_cnn_minimal_to_onnx.py — 将 event_cnn_minimal
(Stoffregen et al., "Reducing the Sim-to-Real Gap for Event Cameras",
ECCV 2020) 的三个预训练模型转换为 ONNX。

用法:
    .venv/bin/python models/convert_event_cnn_minimal_to_onnx.py \
        --model {e2vid_plus,firenet_plus,evflownet} \
        --input /path/to/xxx.pth --output models/xxx.onnx

模型与接口（维度名是 C++ 端 e2vid_inference.h 状态形状规则的契约:
  H2/H4/H8 = 输入的 1/2、1/4、1/8 分辨率; H/W = 全分辨率）:
    e2vid_plus   arch=FlowNet   UNetFlow (ConvLSTM×3)      7入 [voxel,h0,c0,h1,c1,h2,c2]
                                                        → 7出 [image(=ch0), h0',c0',h1',c1',h2',c2']
    firenet_plus arch=FireNet   全分辨率、无步进、ConvGRU×2  3入 [voxel,g1,g2] → 3出 [image,g1',g2']
    evflownet    arch=EVFlowNet 纯前馈 UNet (concat skip)    1入 [voxel] → 1出 [flow(2ch u,v)]

注意:
  - checkpoint 是 ACDA 风格训练存档（arch/state_dict/config，config 被
    pickle 为 parse_config.ConfigParser），加载前注册 parse_config stub。
  - 这些"updated"模型吃 RAW voxel（无归一化）；LegacyNorm 仅用于 legacy
    模型（inference.py --e2vid/--firenet_legacy），与本 GUI 的 voxel 构建
    一致。
  - FlowNet 联合输出 3 通道: image = out[:, 0:1], flow = out[:, 1:3]
    （ref/event_cnn_minimal/model/unet.py UNetFlow.forward:187）。

依赖: torch (CPU)、onnx、numpy（安装于 .venv）。
"""

import argparse
import os
import sys

import numpy as np
import torch  # 必须先于 parse_config stub 导入
import onnx
import onnx.checker


# ---------------------------------------------------------------------------
# checkpoint 加载
# ---------------------------------------------------------------------------

def _install_parse_config_stub():
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


def load_checkpoint(path_to_model, repo_dir):
    """torch.load + parse_config stub；返回 (arch, ckpt, unet_kwargs/args)。"""
    sys.path.insert(0, repo_dir)
    _install_parse_config_stub()
    print(f"Loading model {path_to_model} ...")
    ck = torch.load(path_to_model, map_location="cpu", weights_only=False)
    arch = ck["arch"]
    cfg = ck["config"]._config["arch"]["args"]
    print(f"  arch={arch}")
    return arch, ck, cfg


# ---------------------------------------------------------------------------
# ONNX 导出包装器（显式前向：全部循环状态经函数参数传递，避免追踪时被
# 烘焙成常量——与 convert_hypere2vid_to_onnx.py 同一原则）
# ---------------------------------------------------------------------------

class FlowNetONNXWrapper(torch.nn.Module):
    """E2VID+（UNetFlow, ConvLSTM×3, skip per unet_kwargs）。

    image 取联合输出的通道 0（UNetFlow: image=[:,0:1], flow=[:,1:3]）。
    """

    def __init__(self, unet_kwargs):
        super().__init__()
        from model.unet import UNetFlow  # noqa: E402

        self.unetflow = UNetFlow(dict(unet_kwargs))

    def forward(self, voxel, h0, c0, h1, c1, h2, c2):
        u = self.unetflow
        states = [(h0, c0), (h1, c1), (h2, c2)]
        x = u.head(voxel)
        head = x
        blocks = []
        for i, encoder in enumerate(u.encoders):
            x, state = encoder(x, states[i])
            blocks.append(x)
            states[i] = state
        for resblock in u.resblocks:
            x = resblock(x)
        for i, decoder in enumerate(u.decoders):
            x = decoder(u.skip_ftn(x, blocks[len(states) - i - 1]))
        img_flow = u.pred(u.skip_ftn(x, head))
        image = img_flow[:, 0:1, :, :]
        new_states = []
        for hidden, cell in states:
            new_states.append(hidden)
            new_states.append(cell)
        return (image, *new_states)


class FireNetONNXWrapper(torch.nn.Module):
    """FireNet+（全分辨率，head→ConvGRU→Res→ConvGRU→Res→pred）。

    属性名与 checkpoint 顶层键一一对应（head/G1/R1/G2/R2/pred）。
    ConvGRU 状态为单个张量（非 h/c 对），全分辨率 → 维度名 H/W。
    """

    def __init__(self, num_bins, base_num_channels, kernel_size):
        super().__init__()
        from model.submodules import ConvGRU, ConvLayer, ResidualBlock  # noqa: E402

        padding = kernel_size // 2
        self.head = ConvLayer(num_bins, base_num_channels, kernel_size,
                              padding=padding)
        self.G1 = ConvGRU(base_num_channels, base_num_channels, kernel_size)
        self.R1 = ResidualBlock(base_num_channels, base_num_channels)
        self.G2 = ConvGRU(base_num_channels, base_num_channels, kernel_size)
        self.R2 = ResidualBlock(base_num_channels, base_num_channels)
        self.pred = ConvLayer(base_num_channels, 1, 1, activation=None)

    def forward(self, voxel, g1, g2):
        x = self.head(voxel)
        g1_new = self.G1(x, g1)
        x = self.R1(g1_new)
        g2_new = self.G2(x, g2)
        x = self.R2(g2_new)
        return (self.pred(x), g1_new, g2_new)


class EVFlowNetONNXWrapper(torch.nn.Module):
    """EV-FlowNet（Zhu et al. 2018 架构, Stoffregen 2020 重训练）。

    纯前馈，无状态；输出 2 通道 (u, v) 位移（窗口内像素位移，速度需
    除以窗口时长）。UNet 无 head（首个编码器直接吃 num_bins），
    skip_type='concat'。
    """

    def __init__(self, unet_kwargs):
        super().__init__()
        from model.unet import UNet  # noqa: E402

        # EVFlowNet 类内的硬编码覆盖（ref model.py:205-215），以 checkpoint 为准
        EVFlowNet_kwargs = {
            "base_num_channels": 32,
            "num_encoders": 4,
            "num_residual_blocks": 2,
            "num_output_channels": 2,
            "skip_type": "concat",
            "norm": None,
            "use_upsample_conv": True,
            "kernel_size": 3,
            "channel_multiplier": 2,
        }
        kw = dict(unet_kwargs)
        kw.update(EVFlowNet_kwargs)
        self.unet = UNet(kw)

    def forward(self, voxel):
        return (self.unet.forward(voxel),)


# ---------------------------------------------------------------------------
# 导出
# ---------------------------------------------------------------------------

def _export(wrapper, dummy_args, input_names, output_names, dynamic_axes,
            output_path, opset=17):
    print(f"Exporting to {output_path} ...")
    torch.onnx.export(
        wrapper,
        dummy_args,
        output_path,
        input_names=input_names,
        output_names=output_names,
        dynamic_axes=dynamic_axes,
        opset_version=opset,
        do_constant_folding=True,
        dynamo=False,
    )
    print("  Export complete.")


def export_e2vid_plus(arch_ck, output_path, opset=17):
    kw = arch_ck["config"]._config["arch"]["args"]["unet_kwargs"]
    wrapper = FlowNetONNXWrapper(kw)
    wrapper.load_state_dict(arch_ck["state_dict"])
    wrapper.eval()

    h = w = 256  # /8 整除
    dummy = (torch.zeros(1, kw["num_bins"], h, w),
             torch.zeros(1, 64, h // 2, w // 2), torch.zeros(1, 64, h // 2, w // 2),
             torch.zeros(1, 128, h // 4, w // 4), torch.zeros(1, 128, h // 4, w // 4),
             torch.zeros(1, 256, h // 8, w // 8), torch.zeros(1, 256, h // 8, w // 8))
    names_in = ["event_tensor", "h0", "c0", "h1", "c1", "h2", "c2"]
    names_out = ["image", "h0_new", "c0_new", "h1_new", "c1_new", "h2_new", "c2_new"]
    axes = {"event_tensor": {0: "batch", 2: "H", 3: "W"},
            "image": {0: "batch", 2: "H", 3: "W"}}
    for lvl, (hd, wd) in zip((0, 1, 2), (("H2", "W2"), ("H4", "W4"), ("H8", "W8"))):
        for s in (f"h{lvl}", f"c{lvl}", f"h{lvl}_new", f"c{lvl}_new"):
            axes[s] = {0: "batch", 2: hd, 3: wd}
    _export(wrapper, dummy, names_in, names_out, axes, output_path, opset)


def export_firenet_plus(arch_ck, output_path, opset=17):
    args = arch_ck["config"]._config["arch"]["args"]
    wrapper = FireNetONNXWrapper(args["num_bins"], args["base_num_channels"],
                                 args["kernel_size"])
    wrapper.load_state_dict(arch_ck["state_dict"])
    wrapper.eval()

    h = w = 256  # 全分辨率网络，无整除约束
    dummy = (torch.zeros(1, args["num_bins"], h, w),
             torch.zeros(1, args["base_num_channels"], h, w),
             torch.zeros(1, args["base_num_channels"], h, w))
    names_in = ["event_tensor", "g1", "g2"]
    names_out = ["image", "g1_new", "g2_new"]
    axes = {n: {0: "batch", 2: "H", 3: "W"} for n in names_in + names_out}
    _export(wrapper, dummy, names_in, names_out, axes, output_path, opset)


def export_evflownet(arch_ck, output_path, opset=17):
    kw = arch_ck["config"]._config["arch"]["args"]["unet_kwargs"]
    wrapper = EVFlowNetONNXWrapper(kw)
    wrapper.load_state_dict(arch_ck["state_dict"])
    wrapper.eval()

    h = w = 256  # /16 整除（4 编码器）
    dummy = (torch.zeros(1, kw["num_bins"], h, w),)
    _export(wrapper, dummy, ["event_tensor"], ["flow"],
            {"event_tensor": {0: "batch", 2: "H", 3: "W"},
             "flow": {0: "batch", 2: "H", 3: "W"}},
            output_path, opset)


def verify(output_path):
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
    parser = argparse.ArgumentParser(
        description="Convert event_cnn_minimal .pth checkpoints to ONNX")
    parser.add_argument("--model", required=True,
                        choices=["e2vid_plus", "firenet_plus", "evflownet"])
    parser.add_argument("--input", required=True, help="Path to .pth model")
    parser.add_argument("--output", required=True, help="Path to output .onnx")
    parser.add_argument("--repo-dir", default=None,
                        help="Path to ref/event_cnn_minimal (default: auto)")
    parser.add_argument("--opset", type=int, default=17)
    args = parser.parse_args()

    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    repo_dir = args.repo_dir or os.path.join(repo_root, "ref", "event_cnn_minimal")
    if not os.path.isdir(repo_dir):
        print(f"Error: {repo_dir} not found. Clone event_cnn_minimal first.")
        sys.exit(1)

    arch, ck, _cfg = load_checkpoint(args.input, repo_dir)
    exporters = {
        "e2vid_plus": ("FlowNet", export_e2vid_plus),
        "firenet_plus": ("FireNet", export_firenet_plus),
        "evflownet": ("EVFlowNet", export_evflownet),
    }
    expected_arch, export_fn = exporters[args.model]
    if arch != expected_arch:
        print(f"Error: checkpoint arch is '{arch}', expected '{expected_arch}' "
              f"for --model {args.model}")
        sys.exit(1)
    export_fn(ck, args.output, opset=args.opset)
    verify(args.output)


if __name__ == "__main__":
    main()
