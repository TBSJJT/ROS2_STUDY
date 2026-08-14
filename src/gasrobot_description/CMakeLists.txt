cmake_minimum_required(VERSION 3.8)
project(gasrobot_description)

find_package(ament_cmake REQUIRED)

# 模型、RViz 配置和 Launch 必须保持相对目录，供 xacro include 正确解析。
install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # 自动执行版权、格式和 XML 等 ROS 2 标准检查。
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
