# 气体异常检测、历史位姿补偿和现场图像的统一风险事件。
# header.stamp 表示事件封装完成时间，报警采样时间单独保存在 alarm_time。
std_msgs/Header header

uint8 LEVEL_INFO=0
uint8 LEVEL_WARNING=1
uint8 LEVEL_DANGER=2
uint8 LEVEL_CRITICAL=3

string event_id
string gas_type
float32 concentration
string unit
uint8 risk_level

# 报警时间和用于查询历史 TF 的等效传感器响应延迟。
builtin_interfaces/Time alarm_time
builtin_interfaces/Duration response_delay

# raw_pose 是报警时刻位置，corrected_pose 是历史位姿补偿结果。
geometry_msgs/PoseStamped raw_pose
geometry_msgs/PoseStamped corrected_pose
bool compensation_valid
float32 compensation_distance

# 使用压缩图像降低无线网络传输带宽。
sensor_msgs/CompressedImage scene_image
string description
