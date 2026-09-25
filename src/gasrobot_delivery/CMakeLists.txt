cmake_minimum_required(VERSION 3.8)
project(gasrobot_delivery)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 纯 Python 状态机与 ROS 适配层共用一个包；只安装管理器为运行入口。
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS gasrobot_delivery/manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME delivery_manager)
install(DIRECTORY config docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

# 仿真验收脚本需要独占运行中的 Gazebo，因此不在 colcon test 中自动驱动车辆。
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(delivery_tests test)
endif()

ament_package()
