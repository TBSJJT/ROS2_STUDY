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
检查项目 Python 文档字符串(docstring)是否符合 PEP 257 规范.

PEP 257 是 Python 社区关于文档字符串的官方规范, 规定了: 
- 每个模块、类、函数都应该有文档字符串
- 文档字符串应该使用三重双引号 (triple quotes)
- 文档字符串的第一行应该是一句完整的描述
- 多行文档字符串的格式规范

文档字符串是 Python 代码注释的标准方式, 编辑器和工具可以提取它们
自动生成 API 文档 (如 Sphinx). 良好的文档字符串让代码更易读, 易维护.
运行方式: colcon test 或 pytest test/test_pep257.py
"""

# ament_pep257 是 ROS 2 的文档字符串检查工具
# 它基于 pydocstyle 库, 增加了 ROS 2 特有的配置
from ament_pep257.main import main
import pytest


# @pytest.mark.linter: 给测试打上 "linter" 标签(代码静态检查类)
@pytest.mark.linter
# @pytest.mark.pep257: 给测试打上 "pep257" 标签
# 可以用 pytest -m pep257 只运行文档字符串相关测试
@pytest.mark.pep257
def test_pep257():
    """
    运行 Python 文档字符串规范检查.

    这个测试会扫描项目下所有 Python 源文件, 检查它们的文档字符串
    是否符合 PEP 257 规范.

    --add-ignore 参数说明(忽略以下特定规则): 
    - D202: 不要求文档字符串后有空行
    - D400: 不要求文档字符串第一行以句号结尾
    - D415: 不要求文档字符串第一行以句号结尾(与 D400 类似)

    忽略这些规则是因为某些 ROS 2 项目的文档风格与严格 PEP 257
    有细微差异, 完全遵循反而会降低可读性.

    """
    # main(argv=[...]) 处理参数: 
    #   argv 是一个命令行参数列表, 模拟在终端执行命令
    # 参数含义: 
    #   '.': 检查当前目录
    #   'test': 也检查 test 子目录
    #   '--add-ignore': 添加需要忽略的规则编号
    #   'D202': 忽略"文档字符串后无空行"规则
    #   'D400': 忽略"第一行以句号结尾"规则
    #   'D415': 忽略"第一行以句号结尾"规则(pydocstyle 另一版本)
    rc = main(argv=[
        '.',
        'test',
        '--add-ignore',
        'D202',
        'D400',
        'D415',
    ])
    # 断言退出码 rc 为 0
    # 如果不为 0, 说明有文档字符串不符合规范
    assert rc == 0, 'Found code style errors / warnings'
