cmake_minimum_required(VERSION 3.8)
project(delivery_navigation)

find_package(ament_cmake REQUIRED)

# 导航包只安装通用 Nav2 配置和启动文件；任务状态机由 delivery_tasks 提供。
# 旧巡检节点、路线配置及对应 Python 模块已移除，不再生成其可执行入口。
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

# docs 保留历史研究资料，但不再作为当前运行资源安装。
ament_package()
