cmake_minimum_required(VERSION 3.8)
project(gasrobot_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 安装 Python 任务管理模块和 ros2 run 可执行入口。
ament_python_install_package(${PROJECT_NAME})
install(
  PROGRAMS gasrobot_navigation/inspection_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME inspection_manager
)

# 安装经过实车调参的 Nav2 配置、巡检路线模板与启动文件。
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# 架构说明随软件包安装，便于在 PicoPC 上直接查阅。
install(
  DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # 中文文档字符串不强制套用英文句号规则；保留 Python 风格检查，
  # 并把路线配置测试正式注册到 colcon test。
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_flake8()
  ament_add_pytest_test(route_config_test test/test_route_config.py)
  ament_add_pytest_test(
    map_route_validator_test
    test/test_map_route_validator.py
  )
endif()

ament_package()
