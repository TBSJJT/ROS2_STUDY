cmake_minimum_required(VERSION 3.8)
project(gasrobot_bringup)

find_package(ament_cmake REQUIRED)

# 顶层编排包安装 Launch 和项目自有的 RViz 配置。
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
