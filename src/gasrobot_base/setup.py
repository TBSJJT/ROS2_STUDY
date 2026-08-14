#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gasrobot_base 软件包的安装配置文件。

这个文件告诉 Python 的包管理工具（setuptools）如何安装本软件包：
- 软件包叫什么名字、版本号是多少
- 包含哪些 Python 模块
- 需要安装哪些数据文件（配置文件、参数文件等）
- 对外提供哪些可执行命令

在 ROS 2 中，每个功能包都需要一个 setup.py（Python 包）或 CMakeLists.txt（C++ 包），
这样 colcon build 才能找到并编译它。
"""

# os 模块：提供操作系统相关的功能，这里用来拼接文件路径
import os
# glob 模块：用来查找匹配特定模式的文件，比如查找 config 目录下所有 .yaml 文件
from glob import glob

# setuptools 是 Python 标准的打包工具
# find_packages：自动发现项目中的 Python 包（含有 __init__.py 的目录）
# setup：核心函数，用来描述包的元信息和安装规则
from setuptools import find_packages, setup


# 定义软件包名称，后面多处会引用这个变量
package_name = "gasrobot_base"


# setup() 是 setuptools 的核心函数，所有包信息都通过参数传入
setup(
    # ------------------------------------------------------------------
    # 1. 基本信息
    # ------------------------------------------------------------------
    # name：软件包的名称，其他包可以通过这个名字依赖本包
    name=package_name,
    # version：版本号，遵循语义化版本规范（主版本.次版本.修订号）
    version="0.1.0",
    # maintainer：维护者姓名
    maintainer="book",
    # maintainer_email：维护者邮箱
    maintainer_email="2799572363@qq.com",
    # description：一句话描述这个包的功能
    description="STM32 串口桥接、里程计、IMU 与底盘控制。",
    # license：开源许可证类型，Apache-2.0 是 ROS 2 社区常用许可证
    license="Apache-2.0",

    # ------------------------------------------------------------------
    # 2. 包发现
    # ------------------------------------------------------------------
    # packages：告诉 setuptools 哪些目录是 Python 包
    # find_packages() 自动查找项目下所有包含 __init__.py 的目录
    # exclude=["test", "tests"]：排除 test 目录，测试代码不安装到生产环境
    packages=find_packages(exclude=["test", "tests"]),

    # ------------------------------------------------------------------
    # 3. 数据文件（非 Python 代码文件）
    # ------------------------------------------------------------------
    # data_files：指定需要安装的非 Python 文件
    # 格式是一个列表，每个元素是 (目标安装目录, [源文件列表])
    data_files=[
        # --- 注册 ament 软件包索引 ---
        # 这行让 ROS 2 的工具（如 ros2 pkg list）能发现本软件包
        # share/ament_index/resource_index/packages 是 ROS 2 约定的索引目录
        # resource/ 下需要有一个与包同名的空文件（或标记文件）
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        # --- 安装 package.xml ---
        # package.xml 是 ROS 2 包的元信息文件（类似 package.json）
        # 安装到共享目录，其他工具可以读取
        ("share/" + package_name, ["package.xml"]),
        # --- 安装 YAML 参数文件 ---
        # config/*.yaml 匹配 config 目录下所有 .yaml 文件
        # 安装后路径类似：share/gasrobot_base/config/xxx.yaml
        # launch 文件和命令行可以通过 FindPackageShare 找到这些参数文件
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],

    # ------------------------------------------------------------------
    # 4. 依赖与安装选项
    # ------------------------------------------------------------------
    # install_requires：运行时依赖的其他 Python 包
    # setuptools 是最基础的构建工具，几乎所有包都依赖它
    install_requires=["setuptools"],
    # zip_safe=True：表示本包可以以 zip 压缩格式安装
    # 设为 True 可以加快安装速度
    zip_safe=True,
    # tests_require：运行测试需要额外安装的包
    # pytest 是 Python 最流行的测试框架
    tests_require=["pytest"],

    # ------------------------------------------------------------------
    # 5. 可执行入口点（最重要的部分！）
    # ------------------------------------------------------------------
    # entry_points：定义安装后可以执行的命令行程序
    # console_scripts 格式：命令名 = 包名.模块名:函数名
    entry_points={
        "console_scripts": [
            # 安装后，用户可以通过以下方式运行：
            #   ros2 run gasrobot_base stm32_bridge
            # 效果等同于：
            #   运行 gasrobot_base/stm32_bridge.py 文件中的 main() 函数
            "stm32_bridge = gasrobot_base.stm32_bridge:main",
        ],
    },
)
