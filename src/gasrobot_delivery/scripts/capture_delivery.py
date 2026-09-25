#!/usr/bin/env python3
"""按配送阶段保存真实桌面截图及状态旁录（需 X11 桌面和 python3-pyqt5）。

先手工将 RViz 和 Gazebo 摆放在同一屏幕，再启动本脚本，最后开始配送。
本节点只订阅状态，不发布目标、速度或任务服务，也不暂停 Gazebo。
截图是当前主屏幕的原始像素，不拼接、不重绘；请避免其他窗口遮挡。
示例：python3 src/gasrobot_delivery/scripts/capture_delivery.py --output runs/screenshots
"""

import argparse
from datetime import datetime
import json
from pathlib import Path

from PyQt5.QtWidgets import QApplication
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class ScreenshotRecorder(Node):
    """每个目标的行驶/卸货阶段各记录一次，保留原始状态以便核对图注。"""

    def __init__(self, output, app):
        super().__init__("delivery_screenshot_recorder")
        self.output = output
        self.app = app
        self.current = None
        self.since = 0.0
        self.seen = set()
        self.done = False
        # 状态话题是 transient local：晚启动也能收到当前状态。
        self.create_subscription(
            String,
            "/delivery_manager/status",
            self.on_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

    def on_status(self, message):
        status = json.loads(message.data)
        key = (status["state"], status["target"])
        if self.current is None or key != (
            self.current["state"], self.current["target"]
        ):
            self.since = status["ros_time"]
        self.current = status

    def capture_if_due(self):
        if self.current is None:
            return
        status = self.current
        state, target = status["state"], status["target"]
        key = (state, target)
        if key in self.seen:
            return
        if state not in (
            "IDLE", "NAVIGATING", "SERVICING", "RETURNING", "COMPLETED", "FAILED"
        ):
            return
        # 使用状态携带的 ROS 时间；暂停仿真不会推进截图阶段等待。
        # 跨排前往 S6 等待 30 秒，其他行驶段等待 8 秒，以拍到在途画面。
        # 卸货和终态等待 1 秒，给 RViz/Gazebo 渲染线程留出显示时间。
        if state in ("IDLE", "SERVICING", "COMPLETED", "FAILED"):
            delay = 1
        else:
            delay = 30 if target == "S6" else 8
        if status["ros_time"] - self.since < delay:
            return
        name = f"{len(self.seen) + 1:02d}_{state.lower()}_{target or 'depot'}"
        screen = self.app.primaryScreen()
        if screen is None:
            raise RuntimeError("未发现可截图的主屏幕；请在图形桌面会话运行")
        image = screen.grabWindow(0)
        if image.isNull() or not image.save(str(self.output / (name + ".png"))):
            raise RuntimeError("桌面截图失败")
        # 状态采样和屏幕抓取不是硬件同步，图注不可声称二者毫秒级同步。
        record = dict(
            status,
            file=name + ".png",
            captured_local=datetime.now().astimezone().isoformat(),
            width=image.width(),
            height=image.height(),
        )
        (self.output / (name + ".json")).write_text(
            json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        self.get_logger().info(f"已保存 {name}.png")
        self.seen.add(key)
        self.done = state in ("COMPLETED", "FAILED")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    # 不覆盖此前的截图；一次运行对应一个独立目录。
    args.output.mkdir(parents=True, exist_ok=True)
    if any(args.output.iterdir()):
        parser.error("输出目录必须为空，请为本批次选择新目录")
    app = QApplication([])
    rclpy.init()
    node = ScreenshotRecorder(args.output, app)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            app.processEvents()
            node.capture_if_due()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
