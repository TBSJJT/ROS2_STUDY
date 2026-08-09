# 已与地图坐标系位姿关联的气体读数。
std_msgs/Header header
gasrobot_interfaces/GasReading reading

# 传感器在 reading.header.stamp 时刻的地图位姿。
geometry_msgs/PoseStamped sensor_pose
bool pose_valid
string pose_error
