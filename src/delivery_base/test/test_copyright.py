#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright 2015 Open Source Robotics Foundation, Inc.
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
检查项目源码中是否包含必要的版权声明。

这个测试文件是 ROS 2 代码质量检查体系的一部分。
ROS 2 要求所有源码文件包含 Apache 2.0 许可证的版权声明头。
运行方式：colcon test 或 pytest test/test_copyright.py
"""

# ament_copyright 是 ROS 2 的版权检查工具
# 它会扫描所有源文件，检查是否包含正确的许可证声明
from ament_copyright.main import main
# pytest 是 Python 最流行的测试框架
# 用它提供的装饰器（@pytest.mark.xxx）可以为测试添加标签
import pytest


# @pytest.mark.skip：跳过这个测试，不执行
# reason 参数说明跳过的原因：源码还没有添加版权声明头
# 当源文件补充好版权声明后，删除这行装饰器即可启用测试
@pytest.mark.skip(
    reason='No copyright header has been placed in the generated source file.'
)
# @pytest.mark.copyright：给这个测试打上 "copyright" 标签
# 这样可以用 pytest -m copyright 只运行版权相关测试
@pytest.mark.copyright
# @pytest.mark.linter：给这个测试打上 "linter" 标签
# linter 泛指代码静态检查工具
@pytest.mark.linter
def test_copyright():
    """
    运行 ROS 2 版权声明检查。

    这个测试会调用 ament_copyright 工具扫描当前目录和 test 子目录
    下的所有源文件，检查它们是否包含合法的开源许可证头。

    如果检查通过：返回码 rc == 0，测试通过。
    如果检查失败：返回码 rc != 0，assert 触发 AssertionError，测试失败。

    """
    # main(argv=['.', 'test'])：
    #   - '.' 表示检查当前目录的源码
    #   - 'test' 表示也检查 test 子目录
    # 返回值 rc 是退出码（return code）：
    #   - 0 表示所有文件版权声明正确
    #   - 非 0 表示有问题
    rc = main(argv=['.', 'test'])
    # assert 是 Python 的断言语句：
    #   assert 条件, 错误消息
    #   如果条件为 False，抛出 AssertionError，测试失败
    #   如果条件为 True，什么都不发生，测试继续
    assert rc == 0, 'Found errors'
