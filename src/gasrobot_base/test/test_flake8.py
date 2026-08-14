#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
检查项目 Python 代码是否符合 Flake8 代码风格规范。

Flake8 是 Python 社区最常用的代码风格检查工具，它整合了三个子工具：
- PyFlakes：检查代码逻辑错误（如未使用的变量）
- pycodestyle：检查代码格式是否符合 PEP 8 规范（如缩进、空格、行长度等）
- McCabe：检查代码复杂度（如函数是否过于复杂）

在 ROS 2 中，每个 Python 包都应该通过 Flake8 检查，保证代码风格一致。
运行方式：colcon test 或 pytest test/test_flake8.py
"""

# main_with_errors 是 ament_flake8 提供的函数
# 它与 main 的区别是：
#   - main 只返回退出码
#   - main_with_errors 返回 (退出码, 错误列表) 两个值
# 我们使用后者，这样测试失败时可以打印具体的错误信息
from ament_flake8.main import main_with_errors
import pytest


# @pytest.mark.flake8：给测试打上 "flake8" 标签
# 可以用 pytest -m flake8 只运行 Flake8 相关测试
@pytest.mark.flake8
# @pytest.mark.linter：给测试打上 "linter" 标签
@pytest.mark.linter
def test_flake8():
    """
    运行 Flake8 代码风格检查。

    这个测试会扫描项目下所有 Python 源文件，检查它们是否遵守
    PEP 8 代码风格规范。检查项包括但不限于：
    - 缩进是否使用 4 个空格
    - 每行是否超过 79 个字符
    - 函数和类之间是否有空行
    - 是否有未使用的导入或变量
    - 是否有语法错误

    如果检查通过：退出码 rc == 0，测试通过。
    如果检查失败：退出码 rc != 0，测试失败，并打印所有错误详情。

    """
    # main_with_errors(argv=[]) 处理参数：
    #   argv 是命令行参数列表
    #   [] 表示使用默认配置（不传任何额外参数）
    # 返回值解包（unpacking）：
    #   rc：退出码，0 表示通过
    #   errors：一个字符串列表，每个元素是一条错误描述
    rc, errors = main_with_errors(argv=[])
    # 断言退出码为 0（检查通过）
    # 如果失败，assert 的错误消息会显示：
    #   - 错误总数：len(errors)
    #   - 所有错误的详细描述：用换行符拼接
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
