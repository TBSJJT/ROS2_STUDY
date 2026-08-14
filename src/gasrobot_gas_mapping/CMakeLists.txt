cmake_minimum_required(VERSION 3.8)
project(gasrobot_gas_mapping)

find_package(ament_cmake REQUIRED)

# 同时安装 SLAM 参数、启动文件和实验地图资源。
install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
