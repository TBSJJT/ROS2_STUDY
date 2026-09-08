cmake_minimum_required(VERSION 3.8)
project(gasrobot_simulation)
find_package(ament_cmake REQUIRED)
# 安装仿真运行资源，忽略解释器缓存。
install(DIRECTORY launch config worlds models
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
