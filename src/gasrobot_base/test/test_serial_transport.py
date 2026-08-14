#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
串口传输层（SerialTransport）的单元测试。

本测试文件验证 SerialTransport 类的核心功能：
- 串口连接建立与断开
- 控制线管理（DTR/RTS）
- 缓冲区清空
- 非阻塞读写
- 异常安全关闭

由于测试不应该依赖真实的串口硬件（否则每次测试都需要连接 STM32 底盘），
我们使用"模拟对象"（Fake Object）技术：
创建一个假的串口类 _FakeSerial，它和真实 pyserial 有相同的接口，
但所有操作都在内存中完成，不需要任何硬件。

这种技术也叫做"测试替身"（Test Double），是单元测试的核心技术之一。
"""

# pytest 是 Python 测试框架，这里用它提供的 monkeypatch 夹具
# monkeypatch 可以在运行时替换模块中的对象，用于注入模拟对象
import pytest
# 导入我们要测试的类
from gasrobot_base.serial_transport import SerialTransport


# -----------------------------------------------------------------------
# _FakeSerial：模拟的串口对象
# -----------------------------------------------------------------------
class _FakeSerial:
    """
    模拟 pyserial.Serial 的最小接口，避免测试依赖真实串口硬件。

    这个类只实现了 SerialTransport 实际使用的属性和方法，
    未使用的方法（如 readline、flush 等）不需要实现。
    这样做的好处：
    - 测试可以独立运行，不需要连接 STM32 底盘
    - 测试结果可预测，不受硬件状态影响
    - 测试运行速度快
    - 可以模拟正常情况和异常情况

    """

    def __init__(self) -> None:
        """
        初始化一个假串口对象，所有状态字段都可被测试断言检查。

        属性说明：
        - port: 串口设备路径（如 /dev/ttyUSB0），初始为 None
        - is_open: 串口是否已打开
        - dtr: DTR 控制线状态（True=高电平，False=低电平）
        - rts: RTS 控制线状态
        - in_waiting: 接收缓冲区中待读取的字节数（模拟有 3 个字节等待读取）
        - input_reset: 是否调用过输入缓冲区复位
        - output_reset: 是否调用过输出缓冲区复位
        - closed: 是否调用过 close() 关闭串口

        """
        self.port = None          # 设备路径，初始未知
        self.is_open = False      # 串口默认关闭
        self.dtr = True           # DTR 默认高电平
        self.rts = True           # RTS 默认高电平
        self.in_waiting = 3       # 模拟有 3 个字节等待读取
        self.input_reset = False  # 输入缓冲区未被复位
        self.output_reset = False # 输出缓冲区未被复位
        self.closed = False       # 未被关闭过

    def open(self) -> None:
        """
        模拟打开串口。

        真实操作：操作系统打开设备文件，配置波特率、数据位等参数。
        模拟操作：只是 is_open 标记变为 True。

        """
        self.is_open = True

    def close(self) -> None:
        """
        模拟关闭串口并记录关闭动作。

        真实操作：操作系统关闭设备文件描述符，释放串口资源。
        模拟操作：is_open 变 False，closed 标记记录已关闭。

        """
        self.is_open = False
        self.closed = True

    def reset_input_buffer(self) -> None:
        """
        记录输入缓冲区复位动作。

        真实操作：清空操作系统串口接收缓冲区中的所有待读数据。
        模拟操作：设置标记，让测试可以验证这个方法是否被调用了。

        """
        self.input_reset = True

    def reset_output_buffer(self) -> None:
        """
        记录输出缓冲区复位动作。

        真实操作：清空待发送但尚未发出的数据。
        模拟操作：设置标记，让测试可以验证这个方法是否被调用了。

        """
        self.output_reset = True

    def read(self, size: int) -> bytes:
        """
        按请求长度返回固定测试数据。

        参数:
            size: 请求读取的字节数

        返回:
            固定测试字符串 b"abc" 的前 size 个字节
            例如 size=2 → b"ab"，size=1 → b"a"

        真实操作：从操作系统串口缓冲区读取指定数量的字节。
        模拟操作：返回固定的测试数据 b"abc"。

        """
        return b"abc"[:size]

    def write(self, data: bytes) -> int:
        """
        模拟完整写入并返回字节数。

        参数:
            data: 要写入的字节串

        返回:
            len(data)：始终返回完整长度，表示写入成功

        真实操作：将字节串写入串口发送缓冲区，返回实际写入的字节数。
        模拟操作：始终返回数据长度，模拟写入总是成功。

        """
        return len(data)


# -----------------------------------------------------------------------
# 测试函数
# -----------------------------------------------------------------------
def test_transport_manages_connection_and_nonblocking_io(monkeypatch):
    """
    验证 SerialTransport 管理控制线、缓冲区、读写和关闭状态的完整流程。

    这个测试覆盖了 SerialTransport 的核心功能：
    1. connect()：建立串口连接，配置控制线，清空缓冲区
    2. read_available()：非阻塞读取缓冲区数据
    3. write()：写入数据到串口
    4. close()：安全关闭串口

    参数:
        monkeypatch：pytest 提供的 monkeypatch 夹具（fixture），
                     可以在运行时替换任何模块的属性或对象。
                     这里用它把真实的 pyserial.Serial 替换成 _FakeSerial。

    """
    # --- 步骤 1：创建模拟串口对象 ---
    # _FakeSerial() 创建一个虚假串口实例，所有状态可控、可检查
    connection = _FakeSerial()

    # --- 步骤 2：注入模拟对象 ---
    # monkeypatch.setattr(目标模块.目标类, 替换函数)
    # 这行的意思是：
    #   当 gasrobot_base.serial_transport 模块中的 serial.Serial 被调用时，
    #   不创建真实的串口对象，而是返回我们准备好的 connection
    # lambda **_kwargs: connection 是一个匿名函数：
    #   - **_kwargs 表示接受任意关键字参数，但全部忽略
    #   - 始终返回 connection 这个假对象
    monkeypatch.setattr(
        "gasrobot_base.serial_transport.serial.Serial",
        lambda **_kwargs: connection,
    )

    # --- 步骤 3：创建待测试的 SerialTransport ---
    # 参数：
    #   port="/dev/ttyTEST"：虚拟设备路径（不需要真实存在）
    #   baud=115200：波特率（常用的高速串口波特率）
    #   startup_delay=0.0：启动延时设为 0 以加快测试
    transport = SerialTransport("/dev/ttyTEST", 115200, 0.0)

    # --- 步骤 4：测试 connect() ---
    transport.connect()

    # 断言 1：连接后 is_open 应该为 True
    assert transport.is_open
    # 断言 2：串口设备路径应该被正确设置
    assert connection.port == "/dev/ttyTEST"
    # 断言 3：DTR 控制线应该被置为低电平（False=禁止 DTR）
    # 这是因为 STM32 底盘不需要 DTR/RTS 信号
    assert connection.dtr is False
    # 断言 4：RTS 控制线应该被置为低电平
    assert connection.rts is False
    # 断言 5：输入缓冲区应该被清空了
    assert connection.input_reset
    # 断言 6：输出缓冲区应该被清空了
    assert connection.output_reset

    # --- 步骤 5：测试 read_available() ---
    # 非阻塞读取：in_waiting=3 表示有 3 个字节待读
    # 应该返回 b"abc"（_FakeSerial.read 返回 b"abc"[:3]）
    assert transport.read_available() == b"abc"

    # --- 步骤 6：测试 write() ---
    # 写入 4 个字节，应该返回 4（表示全部写入成功）
    assert transport.write(b"1234") == 4

    # --- 步骤 7：测试 close() ---
    transport.close()
    # 断言：关闭后 is_open 应该为 False
    assert not transport.is_open
    # 断言：底层的 close() 方法被调用了
    assert connection.closed
