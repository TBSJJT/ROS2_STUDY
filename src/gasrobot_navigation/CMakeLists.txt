cmake_minimum_required(VERSION 3.8)
project(gasrobot_navigation)

find_package(ament_cmake REQUIRED)

# 本包不实现导航算法，只安装经过实车调参的 Nav2 配置与启动文件。
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # 使用 ROS 2 标准检查保证 CMake、XML 和 Launch 文件基本质量。
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
