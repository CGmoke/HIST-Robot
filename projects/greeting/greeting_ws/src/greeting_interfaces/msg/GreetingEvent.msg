std_msgs/Header header
uint8 EVENT_LEADER_ARRIVED = 1   # 辅助：领导就位提示（不自动接待）
uint8 EVENT_LEADER_LEAVE   = 2
uint8 EVENT_MANUAL_START   = 3
uint8 EVENT_END            = 4
uint8 event_type
float32 distance_m
string source            # perception | console | panel