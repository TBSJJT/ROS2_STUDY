#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
与 ROS 无关的 pyserial 串口连接生命周期封装.

================================================================================
为什么需要 SerialTransport？

SerialTransport 是对 pyserial 库的一层薄封装, 目的是:
1. 将串口通信与 ROS 2 节点逻辑分离 (关注点分离)
2. 提供统一的生命周期管理 (打开 -> 读写 -> 关闭)
3. 处理控制线 (DTR/RTS) 的特殊要求 (STM32 底盘不需要这些信号)
4. 提供非阻塞读取 (不阻塞 ROS 执行器线程)

================================================================================
串口通信基础知识

串口 (Serial Port) 是一种古老但依然广泛使用的通信方式.
在机器人领域, 常用于上位机 (如树莓派/工控机) 与下位机 (如 STM32 单片机) 通信.

关键参数:
- port:      设备文件路径, Linux 下通常为 /dev/ttyUSB0 或 /dev/ttyACM0
- baudrate:  波特率 (bps, bits per second), 常见有 9600, 115200, 921600 等
              通信双方必须使用相同的波特率
- bytesize:  数据位, 通常为 8 位
- parity:    校验位, 通常不使用 (NONE)
- stopbits:  停止位, 通常为 1 位
- timeout:   读取超时 (秒), 0 表示非阻塞

控制线:
- DTR (Data Terminal Ready): 数据终端就绪
- RTS (Request To Send): 请求发送
某些 USB 转串口芯片 (如 CH340, CP2102) 在 DTR/RTS 状态变化时会复位下位机.
STM32 底盘不需要这些信号, 所以显式禁用它们.

"""

# time 模块: 提供 sleep 函数
import time
# Optional: 类型标注, 表示值可以是某种类型或 None
from typing import Optional

# pyserial: Python 的串口通信库
# 安装: pip install pyserial
import serial
# SerialException: pyserial 的串口异常基类
from serial import SerialException


class SerialTransport:
    """
    负责串口的打开、关闭和非阻塞读写.

    这个类不依赖 ROS, 可以独立测试 (见 test/test_serial_transport.py).

    使用示例:
        transport = SerialTransport("/dev/ttyUSB0", 115200, 0.2)
        transport.connect()
        if transport.is_open:
            data = transport.read_available()
            transport.write(b"hello")
        transport.close()

    """

    def __init__(
        self,
        port: str,
        baud: int,
        startup_delay: float,
    ) -> None:
        """
        保存串口连接参数, 但暂不访问硬件.

        __init__ 只保存参数, 不执行任何硬件操作.
        这样做的好处:
        - 可以创建多个 SerialTransport 对象而不占用串口资源
        - 参数校验可以在 ROS 节点初始化阶段完成, 与硬件访问分离

        参数:
            port:          串口设备路径, 例如 "/dev/ttyUSB0"
            baud:          波特率, 例如 115200
            startup_delay: 串口打开后的稳定延时 (秒)
                           某些 USB 转串口需要短暂延时后通信才稳定
                           设为 0 表示不需要延时

        """
        # 保存连接参数
        self.port = port
        self.baud = baud
        self.startup_delay = startup_delay

        # _serial 保存 pyserial 的 Serial 对象
        # 初始为 None, connect() 成功后才会赋值
        # 前置下划线 _ 表示这是 "私有" 属性, 外部不应直接访问
        self._serial: Optional[serial.Serial] = None

    @property
    def is_open(self) -> bool:
        """
        返回串口是否已经成功打开.

        @property 装饰器让 is_open 可以作为属性访问:
            transport.is_open  (不需要加括号)
        而不是方法调用:
            transport.is_open()

        判断逻辑: self._serial 不为 None 且底层串口确实已打开.
        """
        return self._serial is not None and self._serial.is_open

    def connect(self) -> None:
        """
        按底盘所需串口参数建立连接并清空缓冲区.

        连接过程:
        1. 先关闭已有的连接 (如果存在)
        2. 创建 Serial 对象, 先不指定端口 (port=None)
        3. 关闭 DTR/RTS 控制线
        4. 打开串口
        5. 再次关闭 DTR/RTS (确保它们保持低电平)
        6. 等待 startup_delay 秒, 让硬件稳定
        7. 清空收发缓冲区 (丢弃连接前残留的垃圾数据)

        异常:
            OSError, SerialException: 如果打开串口失败 (设备不存在, 权限不足等)

        """
        # 步骤 1: 关闭已有连接
        # 如果之前已经打开过, 先关闭它
        self.close()

        # 步骤 2: 创建 Serial 对象但不指定端口
        # port=None: 先用 None 创建, 后面再赋值
        # baudrate, bytesize 等参数按标准串口配置
        connection = serial.Serial(
            port=None,                    # 先不指定端口
            baudrate=self.baud,           # 波特率
            bytesize=serial.EIGHTBITS,    # 8 数据位
            parity=serial.PARITY_NONE,    # 无校验
            stopbits=serial.STOPBITS_ONE, # 1 停止位
            timeout=0.0,                  # 读取超时 0 = 非阻塞
            write_timeout=0.1,            # 写入超时 0.1 秒
            xonxoff=False,                # 禁用软件流控 (XON/XOFF)
            rtscts=False,                  # 禁用硬件流控 (RTS/CTS)
            dsrdtr=False,                 # 禁用硬件流控 (DSR/DTR)
        )
        # 然后设置实际的端口路径
        connection.port = self.port

        try:
            # 步骤 3: 打开前先关闭控制线
            self._disable_control_lines(connection)
            # 步骤 4: 打开串口
            connection.open()
            # 步骤 5: 打开后再次关闭控制线
            # 某些芯片在 open() 后会重新拉高 DTR/RTS, 所以需要再次关闭
            self._disable_control_lines(connection)

            # 步骤 6: 稳定延时
            # max(0.0, self.startup_delay): 确保延时不为负数
            time.sleep(max(0.0, self.startup_delay))

            # 步骤 7: 清空缓冲区
            # reset_input_buffer(): 清空接收缓冲区 (丢弃残留数据)
            # reset_output_buffer(): 清空发送缓冲区 (丢弃未发送的数据)
            # 这很重要: 连接前的线路噪声或上次通信的残留数据会影响协议解析
            connection.reset_input_buffer()
            connection.reset_output_buffer()

        except (OSError, SerialException):
            # 连接过程中任何一步失败:
            # 先尝试关闭半打开的串口句柄
            if connection.is_open:
                connection.close()
            # 把原始异常重新抛出, 由上层 (STM32BridgeNode) 处理
            raise

        # 连接成功: 保存 Serial 对象供后续读写使用
        self._serial = connection

    @staticmethod
    def _disable_control_lines(connection: serial.Serial) -> None:
        """
        尽力关闭 DTR/RTS 控制线; 不支持控制线的串口设备可以继续使用.

        为什么需要关闭 DTR/RTS？
        许多 USB 转串口芯片 (CH340, CP2102, FT232 等) 在 DTR 或 RTS 电平变化时,
        会触发下位机的复位引脚 (NRST), 导致 STM32 意外重启.
        关闭这些控制线可以防止误复位.

        参数:
            connection: pyserial 的 Serial 对象

        注意: 这是一个 @staticmethod (静态方法), 不依赖类的实例属性.
        """
        try:
            # DTR = False: 数据终端就绪信号设为低电平
            connection.dtr = False
            # RTS = False: 请求发送信号设为低电平
            connection.rts = False
        except (OSError, SerialException):
            # 某些串口设备 (如虚拟串口) 不支持控制线操作
            # 这种情况下忽略异常, 不传播
            pass

    def close(self) -> None:
        """
        关闭串口并释放 Serial 对象.

        关闭过程:
        1. 取出当前的 Serial 对象
        2. 清空 self._serial (防止重复关闭)
        3. 如果串口是打开的, 调用 close() 关闭

        is_open 属性在关闭后自动变为 False,
        因为 self._serial 已经是 None.
        """
        connection = self._serial
        self._serial = None  # 先清空引用
        if connection is None:
            return  # 已经关闭过了, 直接返回
        try:
            if connection.is_open:
                connection.close()  # 关闭底层串口描述符
        except (OSError, SerialException):
            # 关闭阶段的异常通常不影响程序继续退出
            pass

    def read_available(self) -> bytes:
        """
        非阻塞读取当前接收缓冲区中的全部字节.

        非阻塞 (non-blocking) 的含义:
        - 如果缓冲区有数据, 立即返回数据
        - 如果缓冲区为空, 立即返回空字节串 b""
        - 不会阻塞线程等待数据到达

        为什么需要非阻塞？
        ROS 执行器需要定期检查所有定时器和订阅者.
        如果串口读取阻塞了线程, 其他回调就无法执行.

        返回:
            bytes: 缓冲区中现有的全部数据, 可能为空 (b"")

        """
        # 串口未打开时返回空字节串
        if not self.is_open:
            return b""

        # in_waiting 属性: 接收缓冲区中等待读取的字节数
        # 这是一个非阻塞的属性访问, 立即返回
        waiting = self._serial.in_waiting
        # 如果有数据, 读取全部; 如果为空, 返回空字节串
        return self._serial.read(waiting) if waiting > 0 else b""

    def write(self, data: bytes) -> int:
        """
        写入完整协议帧并返回实际写入的字节数.

        参数:
            data: 要写入的字节串 (通常是一个完整的协议帧)

        返回:
            int: 实际写入的字节数
                 0 表示串口未打开或写入失败

        """
        # 串口未打开时返回 0
        if not self.is_open:
            return 0
        # write() 返回实际写入的字节数
        return self._serial.write(data)
