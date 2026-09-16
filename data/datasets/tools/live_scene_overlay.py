#!/usr/bin/env python3
"""Live scene alignment against replay selection 3's first head-camera frame.

Adapted from the local_live_overlay.py viewer used in the 2026-09-10 Camelo
session. Subscribes to images only; all scene alignment is manual.
"""

import argparse
import math
import os
from pathlib import Path
import time

import numpy as np
from PIL import Image


def save_image(path, array):
    temporary = path.with_suffix(".tmp")
    Image.fromarray(array).save(temporary, format="PNG")
    os.replace(temporary, path)


def main():
    bundle = Path(__file__).resolve().parents[1] / "replay/tower_of_babel_20260915_191802"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ref", type=Path, default=bundle / "start_head.png")
    parser.add_argument("--out", type=Path, default=bundle / "calibration")
    parser.add_argument("--topic", default="/head_camera/zed_node/rgb/color/rect/image")
    parser.add_argument("--seconds", type=float, default=0, help="0 = until window closes")
    parser.add_argument("--headless", action="store_true", help="Save images without a window")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error("--seconds must be finite and >= 0")
    if not args.headless and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        parser.error("请在本机桌面终端运行 cali；无窗口检查可加 --headless --seconds 10")
    try:
        with Image.open(args.ref) as source:
            reference = np.array(source.convert("RGB"))
    except (OSError, ValueError) as error:
        parser.error(f"无法读取参考图 {args.ref}: {error}")
    args.out.mkdir(parents=True, exist_ok=True)
    # Never overwrite a reference if --ref/--out are customized.
    for name in ("live_latest.png", "blend_latest.png"):
        if (args.out / name).resolve() == args.ref.resolve():
            parser.error("参考图不能使用校准输出文件的路径")

    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image as RosImage

    bridge = CvBridge()
    latest = [None, None]
    live = None

    def receive(message):
        latest[:] = [message, time.monotonic()]

    def blend(alpha):
        return np.rint(alpha * live + (1 - alpha) * reference).astype(np.uint8)

    plt = None
    if not args.headless:
        import matplotlib
        matplotlib.use("QtAgg")
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Slider

        fig, axes = plt.subplots(1, 3, figsize=(16, 5))
        fig.canvas.manager.set_window_title("LABS cali - live scene alignment")
        fig.subplots_adjust(bottom=0.23)
        views = []
        for ax, name in zip(axes, ("LIVE", "REFERENCE", "OVERLAY")):
            views.append(ax.imshow(reference if name == "REFERENCE" else np.zeros_like(reference)))
            ax.set_title(name)
            ax.set_axis_off()
        slider = Slider(fig.add_axes([0.3, 0.10, 0.4, 0.035]), "Live opacity", 0, 1, valinit=0.5)
        fig.text(0.5, 0.025, f"Reference: {args.ref.parent.name}/{args.ref.name} | Q / Esc: close",
                 ha="center", fontsize=9)
        title = fig.suptitle("Waiting for camera...")

        def change_opacity(_):
            if live is not None:
                views[2].set_data(blend(slider.val))
                fig.canvas.draw_idle()

        def key_press(event):
            if event.key in ("q", "escape"):
                plt.close(fig)

        slider.on_changed(change_opacity)
        fig.canvas.mpl_connect("key_press_event", key_press)
        plt.ion()
        plt.show()

    rclpy.init()
    node = rclpy.create_node("labs_scene_calibration")
    node.create_subscription(RosImage, args.topic, receive, qos_profile_sensor_data)
    start = time.monotonic()
    next_refresh = start
    count = 0
    print(f"实时叠图校准（只读取相机）\n参考图：{args.ref}\n相机：{args.topic}\n保存位置：{args.out}", flush=True)
    print("拖动 Live opacity 调节透明度；按 Q / Esc、关闭窗口或 Ctrl+C 退出。", flush=True)
    try:
        while rclpy.ok() and (plt is None or plt.fignum_exists(fig.number)):
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if args.seconds > 0 and now - start >= args.seconds:
                break
            if now >= next_refresh:
                next_refresh = now + 1.0
                message, received = latest
                valid = False
                if message is None or now - received > 2.0:
                    status = "NO FRESH CAMERA FRAME - check zed-camera-head"
                    if received is not None:
                        status += f" (last received {now - received:.1f}s ago)"
                else:
                    try:
                        candidate = np.asarray(bridge.imgmsg_to_cv2(message, desired_encoding="rgb8"))
                        if candidate.shape != reference.shape:
                            status = f"SHAPE MISMATCH: live {candidate.shape}, reference {reference.shape}"
                        else:
                            live = candidate
                            alpha = slider.val if plt is not None else 0.5
                            overlay = blend(alpha)
                            # Raw pixel difference (0..255), not an object overlap percentage.
                            difference = np.abs(live.astype(np.int16) - reference.astype(np.int16)).max(axis=2).mean()
                            save_image(args.out / "live_latest.png", live)
                            save_image(args.out / "blend_latest.png", overlay)
                            count += 1
                            valid = True
                            status = f'{time.strftime("%H:%M:%S")} | frame {count} | pixel diff {difference:.1f}/255 (lower = closer)'
                            if plt is not None:
                                views[0].set_data(live)
                                views[2].set_data(overlay)
                    except (ValueError, RuntimeError) as error:
                        status = f"CAMERA IMAGE ERROR: {error}"
                print(status, flush=True)
                if plt is not None:
                    title.set_text(status)
                    title.set_color("black" if valid else "red")
                    fig.canvas.draw_idle()
            if plt is not None:
                plt.pause(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        if plt is not None:
            plt.close("all")
    return 0 if count else 1


if __name__ == "__main__":
    raise SystemExit(main())
