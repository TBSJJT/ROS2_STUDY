#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32 底盘二进制协议的编码、解码与流式拆帧.

帧格式明细

控制帧 (上位机 -> 下位机): 共 9 字节
  Byte 0:     帧头 0x7B
  Byte 1-2:   X 线速度 (mm/s, 有符号 16 位大端)
  Byte 3-4:   Y 线速度 (mm/s, 有符号 16 位大端)
  Byte 5-6:   Z 角速度 (mm/s, 有符号 16 位大端, ROS 方向取反)
  Byte 7:     校验和 = (Byte1~6 之和) & 0xFF
  Byte 8:     帧尾 0x7D

反馈帧 (下位机 -> 上位机): 共 23 字节
  Byte 0:     帧头 0x7B
  Byte 1-2:   X 线速度 (mm/s, 有符号 16 位大端)
  Byte 3-4:   Y 线速度 (mm/s, 有符号 16 位大端)
  Byte 5-6:   Z 角速度 (mm/s, 有符号 16 位大端)
  Byte 7-8:   加速度 X 原始值 (LSB, 有符号 16 位大端)
  Byte 9-10:  加速度 Y 原始值 (LSB, 有符号 16 位大端)
  Byte 11-12: 加速度 Z 原始值 (LSB, 有符号 16 位大端)
  Byte 13-14: 陀螺仪 X 原始值 (LSB, 有符号 16 位大端)
  Byte 15-16: 陀螺仪 Y 原始值 (LSB, 有符号 16 位大端)
  Byte 17-18: 陀螺仪 Z 原始值 (LSB, 有符号 16 位大端)
  Byte 19-20: 航向角 (0.01 度, 有符号 16 位大端)
  Byte 21:    校验和 = (Byte1~20 之和) & 0xFF
  Byte 22:    帧尾 0x7D

================================================================================
关键概念

大端序 (Big-endian):
  高位字节存储在低地址位置.
  例如 16 位整数 0x1234:
    大端: 内存中 [0x12, 0x34]  (先高后低, 人读的自然顺序)
    小端: 内存中 [0x34, 0x12]  (先低后高, x86 CPU 的默认顺序)
  STM32 使用大端序, 所以协议也用大端.

流式解析 (Stream Parsing):
  串口数据是连续的字节流, 没有"帧边界".
  FeedbackStreamParser 维护一个缓冲区, 不断喂入新字节,
  从中寻找完整的帧头和帧尾, 提取出完整的反馈帧.
"""

import math
from dataclasses import dataclass
from typing import List

from gasrobot_base.models import RawFeedback, VelocityCommand

# 0x7B = ASCII 的 '{' 字符
FRAME_HEADER = 0x7B
# 0x7D = ASCII 的 '}' 字符
FRAME_TAIL = 0x7D
# COMMAND_FRAME_SIZE: 控制帧的总字节数 (上位机 -> 下位机)
COMMAND_FRAME_SIZE = 9
# FEEDBACK_FRAME_SIZE: 反馈帧的总字节数 (下位机 -> 上位机)
FEEDBACK_FRAME_SIZE = 23

# YAW_CENTIDEGREES_PER_DEGREE: 航向编码精度系数
# 航向在协议中以 "0.01 度" (厘度) 为单位传输
# 9000 厘度 = 90.00 度, 18000 厘度 = 180.00 度
# 除以 100.0 转换为度, 再用 math.radians() 转为弧度
YAW_CENTIDEGREES_PER_DEGREE = 100.0


# =========================================================================
# ProtocolError: 协议异常
# =========================================================================
class ProtocolError(ValueError):
    """
    表示收到的字节序列不符合底盘协议的规范.
    继承自 ValueError (值错误), 在以下情况抛出:
    - 帧长度不对 (不是预期的 9 或 23 字节)
    - 帧头或帧尾错误 (不是 0x7B 或 0x7D)
    - 校验和错误 (数据传输中损坏)
    """
# VelocityLimits: 速度限幅配置
@dataclass(frozen=True)
class VelocityLimits:
    """
    发送到底盘前使用的速度安全限幅.
    属性:
        linear_x:  X 轴线速度最大绝对值 (m/s)
        linear_y:  Y 轴线速度最大绝对值 (m/s)
        angular_z: Z 轴角速度最大绝对值 (rad/s)
    """
    linear_x: float     # X 轴线速度限幅, m/s
    linear_y: float     # Y 轴线速度限幅, m/s
    angular_z: float    # Z 轴角速度限幅, rad/s
# 底层辅助函数 (私有)
def _finite_or_zero(value: float) -> float:
    """
    math.isfinite(x) 返回:
    - True: x 是普通数字
    - False: x 是 NaN, +Infinity, 或 -Infinity
    参数:
        value: 待过滤的浮点数
    返回:
        有限的值原样返回, NaN/Inf 返回 0.0
    """
    return value if math.isfinite(value) else 0.0


def _clamp(value: float, limit: float) -> float:
    """
    先过滤 NaN/Inf, 再按给定的正负对称范围限幅 (clamp).
    参数:
        value: 待限幅的值
        limit: 正负对称范围 (绝对值上限)
    返回:
        限幅后的值, 范围 [-limit, +limit]
    """
    value = _finite_or_zero(value)
    return max(-limit, min(limit, value))


def _read_i16_be(data: bytes, offset: int) -> int:
    """
    从字节序列的指定偏移量读取一个有符号大端 (Big-endian) 16 位整数.
    参数:
        data:   字节序列
        offset: 读取起始位置 (字节索引, 从 0 开始)
    返回:
        解码后的有符号整数, 范围 [-32768, 32767]
    示例:
        _read_i16_be(b'\x01\x02\x03\x04', 0) -> 0x0102 = 258
        _read_i16_be(b'\xFF\xFE', 0) -> -2 (补码表示)
    """
    return int.from_bytes(
        data[offset:offset + 2],   # 取 offset 开始的 2 个字节
        byteorder="big",           # 大端序: 第一个字节是高位
        signed=True,               # 有符号整数 (使用补码表示负数)
    )


def _write_i16_be(buffer: bytearray, offset: int, value: int) -> None:
    """
    限幅到 16 位有符号整数范围后, 写入大端序 2 字节.

    写入前会先把 value 限制在 int16 的范围 [-32768, 32767] 内.
    这提供了最后一层保护: 即使上游限幅失效, 这里也不会产生溢出的二进制数据.

    参数:
        buffer: 可变的字节数组 (bytearray)
        offset: 写入起始位置 (字节索引)
        value:  要写入的整数值

    """
    # max(-32768, min(32767, int(value))):
    #   先把 value 转为 int, 再限制在 int16 范围内
    #   -32768 是最小值 (0x8000), 32767 是最大值 (0x7FFF)
    bounded = max(-32768, min(32767, int(value)))

    # bounded.to_bytes(2, byteorder="big", signed=True):
    #   把整数编码为 2 字节的大端序有符号格式
    encoded = bounded.to_bytes(2, byteorder="big", signed=True)

    # 把编码后的 2 字节写入 buffer 的指定位置
    buffer[offset:offset + 2] = encoded


# =========================================================================
# 航向解码
# =========================================================================
def decode_yaw_centidegrees(frame: bytes, offset: int) -> float:
    """
    从帧中读取有符号大端 0.01 度航向, 并解码为弧度.

    转换流程:
      1. 读取 int16 值 (厘度单位, 即 0.01 度)
      2. 除以 100.0 -> 得到 "度" 单位
      3. math.radians() -> 度转为弧度

    例如:
      9000 厘度 -> 90.00 度 -> pi/2 弧度
      -9000 厘度 -> -90.00 度 -> -pi/2 弧度

    参数:
        frame:  反馈帧字节序列
        offset: 航向字段的起始字节索引

    返回:
        航向的弧度值

    """
    # 步骤 1-2: 读取 int16 并转为度
    yaw_degrees = (
        _read_i16_be(frame, offset)        # 读取 int16 厘度值
        / YAW_CENTIDEGREES_PER_DEGREE      # 除以 100 -> 度
    )
    # 步骤 3: 度 -> 弧度
    return math.radians(yaw_degrees)


# =========================================================================
# 控制帧编码
# =========================================================================
def encode_velocity_command(
    command: VelocityCommand,
    limits: VelocityLimits,
) -> bytes:
    """
    将 ROS 三轴速度指令编码为 9 字节 STM32 控制帧.

    编码流程:
    1. 三轴速度分别限幅 (防止发送超出范围的值)
    2. 线速度: m/s -> mm/s (乘以 1000)
    3. 角速度: ROS 方向 -> 下位机方向 (取反)
    4. 组装 9 字节帧: [帧头] [vx] [vy] [wz] [校验] [帧尾]

    参数:
        command: VelocityCommand 对象 (ROS 坐标约定)
        limits:  VelocityLimits 对象 (三轴速度安全限幅)

    返回:
        9 字节的 bytes 对象 (控制帧)

    """
    # 步骤 1: 三轴速度限幅
    # 每个轴独立限幅, 确保不超出安全范围
    linear_x = _clamp(command.linear_x, limits.linear_x)
    linear_y = _clamp(command.linear_y, limits.linear_y)
    angular_z = _clamp(command.angular_z, limits.angular_z)

    # 步骤 2: 创建 9 字节的缓冲区
    # bytearray 是可变的字节序列, 可以逐个修改
    frame = bytearray(COMMAND_FRAME_SIZE)

    # 字节 0: 帧头
    frame[0] = FRAME_HEADER

    # 字节 1-2: X 线速度
    # m/s -> mm/s: 乘以 1000.0, 四舍五入到整数
    # 例如 1.5 m/s -> 1500 mm/s
    _write_i16_be(frame, 1, round(linear_x * 1000.0))

    # 字节 3-4: Y 线速度
    _write_i16_be(frame, 3, round(linear_y * 1000.0))

    # 字节 5-6: Z 角速度 (旋转方向转换)
    # 注意负号! ROS 的逆时针为正, 下位机的约定方向可能相反
    # 在协议层统一完成符号转换, 其他模块不需要关心
    # 同样单位换算: rad/s -> mrad/s (乘以 1000)
    _write_i16_be(frame, 5, round(-angular_z * 1000.0))

    # 字节 7: 校验和
    # sum(frame[1:7]): 字节 1-6 的所有值相加
    # & 0xFF: 取低 8 位 (等价于 modulo 256)
    frame[7] = sum(frame[1:7]) & 0xFF

    # 字节 8: 帧尾
    frame[8] = FRAME_TAIL

    # bytes(frame): 把可变的 bytearray 转为不可变的 bytes
    return bytes(frame)


# =========================================================================
# 反馈帧解码
# =========================================================================
def decode_feedback_frame(frame: bytes) -> RawFeedback:
    """
    校验并解码一帧 23 字节 STM32 底盘反馈.

    解码流程:
    1. 检查帧长度是否为 23 字节
    2. 检查帧头 (字节 0) 是否为 0x7B
    3. 检查帧尾 (字节 22) 是否为 0x7D
    4. 检查校验和 (字节 21) 是否匹配
    5. 逐一解码各数据字段
    6. 组装并返回 RawFeedback 对象

    只有通过全部 4 项检查的帧才会被解码.
    任何一项不通过都会抛出 ProtocolError, 该帧被丢弃.

    参数:
        frame: 恰好 23 字节的反馈帧 (不含帧头前和帧尾后的数据)

    返回:
        RawFeedback 对象 (不可变数据类)

    异常:
        ProtocolError: 如果帧有任何格式问题

    """
    # ---------- 校验阶段 ----------
    # 校验 1: 检查帧长度
    # 这应该在 FeedbackStreamParser 中保证, 但这里再检查一次作为保险
    if len(frame) != FEEDBACK_FRAME_SIZE:
        raise ProtocolError("反馈帧长度不正确")

    # 校验 2: 检查帧头
    # 第一个字节必须是 0x7B
    if frame[0] != FRAME_HEADER:
        raise ProtocolError("反馈帧头不正确")

    # 校验 3: 检查帧尾
    # frame[-1] 是最后一个字节 (Python 的负索引)
    # 它必须是 0x7D
    if frame[-1] != FRAME_TAIL:
        raise ProtocolError("反馈帧尾不正确")

    # 校验 4: 检查校验和
    # frame[21] 是校验和字段
    # sum(frame[1:21]) 计算字节 1-20 之和 (不含帧头和校验字节)
    # & 0xFF 取低 8 位
    if frame[21] != (sum(frame[1:21]) & 0xFF):
        raise ProtocolError("反馈帧校验和不正确")

    # ---------- 解码阶段 ----------
    # 所有校验通过后才解码字段
    # 字段解码与编码对称, 但多了 IMU 原始数据 (保留为 LSB 计数值)
    return RawFeedback(
        # 速度字段: mm/s -> m/s (除以 1000.0)
        linear_x=_read_i16_be(frame, 1) / 1000.0,    # X 线速度
        linear_y=_read_i16_be(frame, 3) / 1000.0,    # Y 线速度
        angular_z=-_read_i16_be(frame, 5) / 1000.0,   # Z 角速度 (取反)
        # 航向: 解码为弧度
        yaw=decode_yaw_centidegrees(frame, 19),
        # IMU 原始数据: 保留为 LSB 计数值, 由 ImuConverter 进一步转换
        acceleration_raw=(
            _read_i16_be(frame, 7),    # 加速度 X
            _read_i16_be(frame, 9),    # 加速度 Y
            _read_i16_be(frame, 11),   # 加速度 Z
        ),
        gyroscope_raw=(
            _read_i16_be(frame, 13),   # 陀螺仪 X
            _read_i16_be(frame, 15),   # 陀螺仪 Y
            _read_i16_be(frame, 17),   # 陀螺仪 Z
        ),
    )


# =========================================================================
# FeedbackStreamParser: 流式帧解析器
# =========================================================================
class FeedbackStreamParser:
    """
    从任意分片的串口字节流中恢复完整的反馈帧.

    串口通信的特点:
    - 数据以字节流的形式到达, 没有"消息边界"
    - 一次 read() 可能返回: 半帧、多个完整帧、或混合
    - 可能有线路噪声 (随机字节)

    FeedbackStreamParser 维护一个内部缓冲区, 不断喂入新字节,
    从中寻找帧头 0x7B, 提取足够长度的数据, 交由 decode_feedback_frame
    校验和解析.

    解析策略:
    1. 追加新数据到缓冲区
    2. 搜索第一个帧头 0x7B 的位置
    3. 丢弃帧头之前的字节 (视为噪声)
    4. 如果缓冲区中数据不足 23 字节, 等待更多数据
    5. 取 23 字节尝试解码:
       - 成功 -> 移除这 23 字节, 返回反馈帧
       - 失败 -> 移除 1 字节, 回到步骤 2 (逐字节滑窗搜索)
    6. 如果缓冲区过大 (超过 max_buffer_size), 清空并报错
       (防止异常数据无限占用内存)

    """

    def __init__(self, max_buffer_size: int = 2048) -> None:
        """
        创建带有最大缓存保护的流式解析器.

        参数:
            max_buffer_size: 缓冲区的最大字节数 (默认 2048)
                             超过此限制时清空缓冲区, 防止异常数据无限增长
                             23 字节一帧, 50Hz -> 1150 B/s
                             2048 字节足够缓冲约 1.8 秒的数据

        """
        # _buffer: 内部缓冲区, 存储尚未成帧的字节
        # bytearray 是可变的, 支持高效的追加和删除操作
        self._buffer = bytearray()

        # max_buffer_size: 缓冲区最大大小
        self.max_buffer_size = max_buffer_size

        # bad_frame_count: 坏帧计数器
        # 记录: 丢弃的噪声字节次数 + 校验失败的帧数 + 缓冲区清空次数
        # 用于 _print_status() 中的状态诊断
        self.bad_frame_count = 0

    def clear(self) -> None:
        """
        清空尚未组成完整帧的内部缓冲区.

        在以下场景调用:
        - 串口重连后 (丢弃断线前的残留数据)
        - 协议同步丢失后 (需要从干净状态重新开始)

        """
        self._buffer.clear()

    def feed(self, data: bytes) -> List[RawFeedback]:
        """
        追加新的串口数据并返回本次解析出的全部完整反馈帧.

        这是解析器的主要入口, 每次串口读取后被调用.

        参数:
            data: 新收到的字节数据 (可能为空)

        返回:
            List[RawFeedback]: 本次解析出的所有完整反馈帧列表
                               可能为空列表 (数据不足或都是坏帧)

        """
        # 步骤 1: 追加新数据到缓冲区
        if data:
            self._buffer.extend(data)

        feedback_items = []  # 收集解析出的帧
        # header: 帧头字节的 bytes 形式, 用于在缓冲区中搜索
        header = bytes((FRAME_HEADER,))

        # 步骤 2: 循环解析, 直到缓冲区中没有完整帧可用
        while self._buffer:
            # --- 2a: 搜索帧头 ---
            # find() 返回 header 在缓冲区中首次出现的索引
            # 如果找不到, 返回 -1
            header_index = self._buffer.find(header)

            if header_index < 0:
                # 整个缓冲区都没有帧头 -> 全部是噪声, 清空
                self.bad_frame_count += 1
                self._buffer.clear()
                break  # 退出循环

            if header_index > 0:
                # 帧头不在开头 -> 之前有噪声字节, 丢弃
                del self._buffer[:header_index]
                self.bad_frame_count += 1
                # 注意: 删除后继续循环, 现在帧头在位置 0

            # --- 2b: 检查数据是否足够 ---
            if len(self._buffer) < FEEDBACK_FRAME_SIZE:
                # 数据不够 23 字节 -> 等待更多数据
                break  # 退出循环, 等待下一次 feed()

            # --- 2c: 尝试解码 ---
            # 取前 23 字节作为一个候选帧
            frame = bytes(self._buffer[:FEEDBACK_FRAME_SIZE])

            try:
                # 尝试解码 (包括校验和验证)
                feedback = decode_feedback_frame(frame)
            except ProtocolError:
                # 解码失败 (校验和错误等)
                # 只滑动 1 个字节, 继续搜索下一个可能的帧头
                # 为什么要滑动 1 字节而不是 23 字节?
                # 因为可能有一个 "假帧头": 数据字节恰好等于 0x7B,
                # 而后面的数据不是合法的帧
                del self._buffer[0]
                self.bad_frame_count += 1
                continue  # 回到循环开头, 重新搜索帧头

            # --- 2d: 解码成功 ---
            # 从缓冲区中移除这 23 字节
            del self._buffer[:FEEDBACK_FRAME_SIZE]
            # 收集解码后的帧
            feedback_items.append(feedback)
            # 继续循环, 检查缓冲区中是否还有更多完整帧

        # 步骤 3: 缓冲区溢出保护
        # 如果缓冲区过大, 说明长时间无法成功解析帧
        # 可能是波特率不匹配或数据完全损坏
        # 清空缓冲区防止内存无限增长
        if len(self._buffer) > self.max_buffer_size:
            self._buffer.clear()
            self.bad_frame_count += 1

        return feedback_items
