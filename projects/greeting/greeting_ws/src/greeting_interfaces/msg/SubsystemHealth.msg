std_msgs/Header header
bool robot_control_alive
bool arm_cmd_has_subscriber
bool chassis_rest_ok
bool audio_ok
bool script_version_ok
uint8 slam_status        # 0 正常 / 1 丢失 / 2 精度下降 / 3 异常 / 5 等待重定位 / 6 重定位中
uint8 dtc_level          # 0 OK / 1 INFO / 2 WARNING / 3 ERROR / 4 FATAL
string detail