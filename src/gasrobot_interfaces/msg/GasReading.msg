# 单个气体传感器的一次原始或已换算采样。
# 时间戳必须对应传感器采样时刻，是历史位姿查询的时间基准。
std_msgs/Header header

uint8 STATUS_OK=0
uint8 STATUS_WARMING_UP=1
uint8 STATUS_CALIBRATING=2
uint8 STATUS_FAULT=3

string sensor_id
string gas_type
float32 concentration
string unit
float32 temperature_celsius
float32 relative_humidity
uint8 status
bool valid
