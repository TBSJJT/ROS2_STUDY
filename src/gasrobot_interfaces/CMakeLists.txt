cmake_minimum_required(VERSION 3.8)
project(gasrobot_interfaces)

find_package(ament_cmake REQUIRED)

# 项目自有的消息、服务和动作接口统一放在此软件包中。
# 定义首个稳定接口后，再添加 rosidl 生成规则。
ament_package()
