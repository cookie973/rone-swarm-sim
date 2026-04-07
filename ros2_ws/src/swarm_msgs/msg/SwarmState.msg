# 蜂群状态消息 — 每架无人机周期性广播
uint8                    drone_id      # 无人机编号 (1-6)
geometry_msgs/Point      position      # ENU 位置 (m)
geometry_msgs/Vector3    velocity      # ENU 速度 (m/s)
float64                  heading       # 航向角 (rad, ENU)
builtin_interfaces/Time  timestamp     # 消息时间戳
