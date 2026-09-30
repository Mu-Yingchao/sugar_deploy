"""只读检查真机 RealSense：枚举设备、读取彩色帧并打印内参。

这个脚本不导入 Unitree SDK、不连接 DDS，也不会发送任何电机指令。应在相机 USB
实际连接的计算机上运行；G1 头部相机接到颈后 USB-C 时，通常应在机器人 PC2 上运行。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time

import numpy as np
import tyro


@dataclass
class Args:
    width: int = 640
    height: int = 480
    fps: int = 30
    frame_count: int = 30
    serial: str | None = None
    """有多台 RealSense 时指定序列号；单台时可省略。"""
    list_only: bool = False
    """只枚举设备，不启动彩色流。"""
    output: Path | None = None
    """可选：把最后一帧保存成 PNG，便于确认视野和朝向。"""


def _device_info(device, rs) -> tuple[str, str, str]:
    def read(field) -> str:
        return device.get_info(field) if device.supports(field) else "unknown"

    return (
        read(rs.camera_info.name),
        read(rs.camera_info.serial_number),
        read(rs.camera_info.usb_type_descriptor),
    )


def main(args: Args) -> None:
    if args.frame_count <= 0:
        raise SystemExit("--frame-count 必须大于 0")
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise SystemExit(
            "缺少 pyrealsense2；请在相机实际连接的机器上安装："
            "python -m pip install pyrealsense2"
        ) from exc

    devices = list(rs.context().query_devices())
    if not devices:
        raise SystemExit(
            "没有枚举到 RealSense。检查脚本是否运行在相机实际连接的机器、USB-C 是否为"
            "数据线/主机口，以及 lsusb 是否能看到 Intel RealSense。"
        )

    serials: list[str] = []
    for index, device in enumerate(devices):
        name, serial, usb_type = _device_info(device, rs)
        serials.append(serial)
        print(
            f"[camera-probe] device[{index}]: name={name!r}, serial={serial!r}, "
            f"usb={usb_type!r}",
            flush=True,
        )
    if args.list_only:
        return

    selected_serial = args.serial
    if selected_serial is None:
        if len(devices) != 1:
            raise SystemExit("检测到多台 RealSense；请用 --serial 明确选择，禁止依赖枚举顺序")
        selected_serial = serials[0]
    elif selected_serial not in serials:
        raise SystemExit(f"指定序列号 {selected_serial!r} 不在枚举结果中")

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(selected_serial)
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.rgb8, args.fps)
    profile = pipeline.start(config)
    last_rgb: np.ndarray | None = None
    timestamps: list[float] = []
    try:
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_stream.get_intrinsics()
        print(
            "[camera-probe] intrinsics: "
            f"width={intr.width}, height={intr.height}, fx={intr.fx:.3f}, fy={intr.fy:.3f}, "
            f"cx={intr.ppx:.3f}, cy={intr.ppy:.3f}",
            flush=True,
        )
        for _ in range(args.frame_count):
            frames = pipeline.wait_for_frames(5000)
            color = frames.get_color_frame()
            if not color:
                raise RuntimeError("RealSense 返回的 frameset 中没有彩色帧")
            rgb = np.asanyarray(color.get_data())
            expected = (args.height, args.width, 3)
            if rgb.shape != expected or rgb.dtype != np.uint8:
                raise RuntimeError(
                    f"彩色帧格式异常：shape={rgb.shape}, dtype={rgb.dtype}，预期 {expected}/uint8"
                )
            last_rgb = rgb.copy()
            timestamps.append(time.monotonic())
    finally:
        pipeline.stop()

    elapsed = timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0.0
    measured_fps = (len(timestamps) - 1) / elapsed if elapsed > 0 else float("nan")
    assert last_rgb is not None
    print(
        f"[camera-probe] PASS: frames={len(timestamps)}, measured_fps={measured_fps:.2f}, "
        f"pixel_range=[{int(last_rgb.min())},{int(last_rgb.max())}]",
        flush=True,
    )
    if args.output is not None:
        from PIL import Image

        args.output.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(last_rgb, mode="RGB").save(args.output)
        print(f"[camera-probe] 最后一帧已保存：{args.output.resolve()}", flush=True)


if __name__ == "__main__":
    main(tyro.cli(Args))
