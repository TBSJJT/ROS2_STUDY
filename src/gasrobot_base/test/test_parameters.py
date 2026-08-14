#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
底盘桥接参数声明、读取与校验的单元测试。

本测试文件验证 BridgeConfig 类的核心功能：
- 默认配置的合理性（所有默认值能通过校验）
- 参数校验能拒绝危险值（如波特率为 0、发送频率为 0、死区为负等）
- 空字符串参数被拒绝（如串口设备名为空、话题名为空等）

参数校验是 ROS 2 节点的第一道防线：
在节点正式运行之前，就应该发现配置错误并给出清晰的错误信息，
避免在运行中因为不合理参数导致不可预期的行为（如底盘失控）。
"""

# pytest 提供测试框架、参数化测试和异常断言
import pytest

# BridgeConfig 是被测试的配置类
from gasrobot_base.parameters import BridgeConfig


# -----------------------------------------------------------------------
# @pytest.mark.parametrize：参数化测试
# -----------------------------------------------------------------------
# 这是一个非常强大的 pytest 功能：
# 同一个测试函数可以用不同的输入参数运行多次，
# 每次运行就像是一个独立的测试用例。
#
# 参数说明：
#   "overrides, expected_name"：两个参数名
#   [(...), (...), ...]：参数值列表，每个元组对应一次测试运行
#
# 这个测试会运行 4 次，分别测试不同参数的校验：
#   1. baud=0       → 期望错误信息包含 "baud"
#   2. tx_rate=0.0  → 期望错误信息包含 "tx_rate"
#   3. gyro_z_deadband=-0.1 → 期望错误信息包含 "gyro_z_deadband"
#   4. port=""       → 期望错误信息包含 "port"
@pytest.mark.parametrize(
    "overrides, expected_name",
    [
        ({"baud": 0}, "baud"),                         # 波特率不能为 0
        ({"tx_rate": 0.0}, "tx_rate"),                # 发送频率不能为 0
        ({"gyro_z_deadband": -0.1}, "gyro_z_deadband"), # 死区不能为负
        ({"port": ""}, "port"),                       # 端口名不能为空
    ],
)
def test_invalid_configuration_is_rejected(overrides, expected_name):
    """
    验证危险或无效参数能够在启动阶段被拒绝。

    参数校验的重要性：
    1. baud=0：波特率为 0 意味着串口无法通信，应该在启动时报错
    2. tx_rate=0：发送频率为 0 意味着不向底盘发送任何速度指令，
       底盘收不到指令会进入不安全状态
    3. gyro_z_deadband < 0：负的死区值没有物理意义
    4. port=""：空设备名无法打开串口

    测试逻辑：
    1. 先获取默认配置（默认配置本身是合法的）
    2. 用 overrides 字典替换要测试的字段值
    3. 调用 validate() 校验
    4. 期望它抛出 ValueError 异常，且异常信息包含 expected_name

    参数:
        overrides: 要覆盖的配置字段字典，如 {"baud": 0}
        expected_name: 期望在异常信息中出现的字段名

    """
    # --- 步骤 1：获取默认配置 ---
    # BridgeConfig() 不带参数调用，使用所有默认值（如 port="/dev/ttyUSB0"）
    # .__dict__ 把数据类实例转为普通字典，方便后续修改
    # 默认配置本身是合法的，所以 BridgeConfig() 不会报错
    values = BridgeConfig().__dict__.copy()

    # --- 步骤 2：覆盖要测试的参数 ---
    # .update(overrides) 用测试参数替换字典中的对应值
    # 例如 overrides={"baud": 0} 会把 baud 从 115200 改为 0
    values.update(overrides)

    # --- 步骤 3-4：断言校验会抛出 ValueError ---
    # pytest.raises(异常类型, match=匹配模式)：
    #   这是一个上下文管理器，检查 with 块中的代码是否抛出指定异常
    #   match 参数是一个正则表达式，检查异常消息是否匹配
    #   如果没抛异常或异常类型不对，测试失败
    with pytest.raises(ValueError, match=expected_name):
        # BridgeConfig(**values)：用 ** 语法把字典解包为关键字参数
        # 效果等同于 BridgeConfig(port="...", baud=0, ...)
        # .validate()：运行参数校验逻辑
        BridgeConfig(**values).validate()


# -----------------------------------------------------------------------
# 测试 2：默认配置有效性
# -----------------------------------------------------------------------
def test_default_configuration_is_valid():
    """
    验证代码内置的默认参数能够通过完整校验。

    这个测试确保：
    1. BridgeConfig 的所有默认值都是合理的
    2. 用户在不提供任何参数的情况下（即只改端口），
       使用默认值也能正常启动节点
    3. 如果某次代码修改不小心把默认值改坏了，
       这个测试会立刻发现问题

    测试非常简单：
    - 用默认参数创建 BridgeConfig 实例
    - 调用 validate() 方法
    - 如果没有抛出异常，测试通过

    """
    # BridgeConfig() 不带参数 = 所有字段使用类定义中的默认值
    # .validate() 检查所有参数是否合法
    # 如果不合法会抛出 ValueError，测试自动失败
    # 如果合法，什么都不会发生，测试通过
    BridgeConfig().validate()
