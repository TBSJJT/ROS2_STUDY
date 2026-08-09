cmake_minimum_required(VERSION 3.8)
project(gasrobot_bringup)

find_package(ament_cmake REQUIRED)

# 顶层编排包只安装 Launch，不承载运行节点。
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
