#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""保留稳定可执行名称的 STM32 桥接薄入口。"""

from typing import Optional

import rclpy

from gasrobot_base.node import STM32BridgeNode


# 保留原类名，避免已有导入代码立即失效。
STM32Bridge = STM32BridgeNode


def main(args=None) -> None:
    """启动 STM32 底盘桥接节点。"""

    rclpy.init(args=args)
    node: Optional[STM32BridgeNode] = None
    try:
        node = STM32BridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop_and_close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
