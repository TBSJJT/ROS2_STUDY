#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# typing.Optional：类型标注，表示一个值可以是某种类型或 None
from typing import Optional

# rclpy 是 ROS 2 的 Python 客户端库
# 所有 ROS 2 Python 节点都需要通过它来初始化和运行
import rclpy

# 从 node 模块导入实际的节点实现类
from gasrobot_base.node import STM32BridgeNode


# -----------------------------------------------------------------------
# 向后兼容别名
# -----------------------------------------------------------------------
# 旧版本的类名叫 STM32Bridge，现在改名为 STM32BridgeNode。
# 保留这个别名，让已有导入代码不会立即失效。
# 例如旧的 `from gasrobot_base.stm32_bridge import STM32Bridge` 仍然可用。
STM32Bridge = STM32BridgeNode


# -----------------------------------------------------------------------
# main 函数：程序入口
# -----------------------------------------------------------------------
def main(args=None) -> None:
    """
    启动 STM32 底盘桥接节点。

    这是 setup.py 中 entry_points 指定的入口函数。
    执行流程：
    1. rclpy.init()：初始化 ROS 2 客户端库
       - 解析命令行参数（如 --ros-args）
       - 初始化日志系统
       - 初始化节点间通信基础设施
    2. 创建 STM32BridgeNode 实例
       - 读取参数（串口路径、波特率、话题名等）
       - 尝试打开串口连接 STM32 底盘
       - 创建发布者、订阅者和定时器
    3. rclpy.spin()：进入事件循环
       - 不断检查是否有新消息到达
       - 调用对应的回调函数处理
       - 定时器到期时执行周期任务
       - 直到节点被关闭或收到中断信号
    4. 异常处理：
       - KeyboardInterrupt：用户按下 Ctrl+C，这是正常退出方式
       - 其他异常：在 finally 块中尽力安全退出

    参数:
        args: 命令行参数列表，默认 None 表示使用 sys.argv
              ROS 2 会从中提取 --ros-args 相关参数

    """
    # rclpy.init()：初始化 ROS 2 客户端库
    # 必须在创建任何节点之前调用
    # 如果程序退出前不调用 rclpy.shutdown()，资源可能无法正确释放
    rclpy.init(args=args)

    # node 变量需要先声明为 Optional 类型
    # 初始为 None，在 try 块中创建实际节点
    # 这样 finally 块可以安全地检查 node 是否为 None
    node: Optional[STM32BridgeNode] = None

    try:
        # 创建节点实例
        # 构造过程中会：
        #   - 调用父类 Node.__init__("stm32_bridge") 注册节点名
        #   - 读取所有 ROS 参数
        #   - 尝试通过 pyserial 连接 STM32 底盘
        #   - 创建定时器和话题接口
        # 如果串口打开失败，节点仍然创建成功（后续会自动重连）
        node = STM32BridgeNode()

        # rclpy.spin(node)：进入 ROS 2 事件循环
        # 这个函数会一直运行（阻塞当前线程），直到：
        #   - 节点被销毁
        #   - 收到 SIGINT（Ctrl+C）或 SIGTERM 信号
        #   - 所有 executor 被关闭
        # 在 spin 期间，回调函数（定时器回调、订阅回调等）会被自动调用
        rclpy.spin(node)

    except KeyboardInterrupt:
        # 用户按下 Ctrl+C 属于正常退出，不需要打印异常堆栈
        # pass 表示什么都不做，继续执行 finally 块
        pass

    finally:
        # finally 块的特点：无论是否发生异常，一定会执行
        # 这确保了节点在退出时能够安全地停机

        # 如果节点创建成功（不为 None），执行安全退出流程
        if node is not None:
            # 1. 向 STM32 底盘发送多帧停车指令
            #    即使串口已断开也会尽力尝试
            node.stop_and_close()
            # 2. 销毁 ROS 2 节点
            #    取消所有定时器、关闭所有发布者和订阅者
            node.destroy_node()

        # rclpy.ok()：检查 ROS 2 是否还在正常运行
        # 如果在 rclpy.shutdown() 已经被调用后再调用一次会出错
        # 但这个检查可以防止那种情况
        if rclpy.ok():
            # 3. 关闭 ROS 2 客户端库
            #    释放所有通信资源，清理中间件连接
            rclpy.shutdown()


# Python 的特殊变量 __name__：
#   - 当文件被直接运行（python stm32_bridge.py）：__name__ = "__main__"
#   - 当文件被导入（import stm32_bridge）：__name__ = "stm32_bridge"
#
# 这个判断让文件既可以被 ros2 run 启动（通过 entry_points），
# 也可以直接用 Python 运行（用于调试）。
if __name__ == "__main__":
    main()
