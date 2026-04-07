# QGC 集群演示一页稿

## 演示目标

用 QGroundControl 地图任务，直接驱动 6 架无人机集群完成以下 4 类动作：

1. 编队整体飞向目标点
2. 多目标搜索分配
3. 单点盘旋搜索
4. 原点返航降落

## 演示前检查

- Gazebo / ArduPilot SITL 已启动
- 6 个 `mavlink_tx` 已启动
- `formation_commander` 已启动
- `qgc_bridge` 已启动
- `/uav1/uav/navsatfix` 正常
- 6 架机已经起飞到约 5 m，并处于 `GUIDED`

## QGC 地图任务怎么画

### 展示 1：编队整体飞向目标点

QGC Mission 中只放 1 个 `Waypoint`

结果：

- 桥接到 `/swarm/mission_center`
- 编队整体平移到目标区域
- 队形保持不变

### 展示 2：多目标搜索

QGC Mission 中放多个 `Waypoint`

结果：

- 桥接到 `/swarm/search_mission`
- 每个点变成一个搜索目标
- 集群按权重自动分配无人机数量

参数：

- `param1` = 搜索点权重
- 例如：`3.0 / 1.5 / 0.8`

### 展示 3：单点盘旋搜索

QGC Mission 中放 1 个 `Loiter Unlimited`

结果：

- 桥接到 `/swarm/search_orbit`
- 编队飞向中心点后进入盘旋搜索

参数：

- `param3` = 盘旋半径，单位米
- 不填默认 `12.0`

### 展示 4：原点返航降落

QGC Mission 中只放 1 个 `Return to Launch`

结果：

- 桥接到 `/swarm/formation = origin_land`
- 集群回原点并降落

## 已在线验证的关键结论

- 单航点 mission：桥接返回 `MISSION_REQUEST_INT -> MISSION_ACK`
- 飞行中单航点 mission：12 秒内编队中心实际位移约 `(5.01 m, 5.31 m)`
- 多航点 mission：成功桥接为 `/swarm/search_mission`，权重随 `param1` 保留
- `Loiter Unlimited`：成功桥接为 `/swarm/search_orbit`
- 单个 `RTL`：成功桥接为 `/swarm/formation = origin_land`

## 演示顺序建议

1. 起飞到 5 m
2. 在 QGC 上下发 1 个 `Waypoint`
3. 观察编队整体平移
4. 清空任务
5. 下发 3 个 `Waypoint`，展示多目标搜索分配
6. 清空任务
7. 下发 1 个 `Loiter Unlimited`
8. 观察盘旋搜索
9. 最后下发 1 个 `Return to Launch`

## 演示时的口径

- QGC 负责地图交互和任务编辑
- `qgc_bridge` 负责把 QGC mission 翻译成集群语义
- `formation_commander` 负责真正的集群控制、队形保持和任务执行
- 这不是单机 Mission 的简单广播，而是“地图任务 -> 集群行为”的语义桥接

## 不要这样演示

- 不要把 `Waypoint`、`Loiter`、`RTL` 混成一个复杂长 Mission
- 不要在地面态直接展示任务执行效果
- 不要把 QGC 说成直接控制 6 架机逐项飞航线

## 最短答辩总结

这套系统已经实现：

- QGC 地图任务输入
- MAVLink mission 协议桥接
- 经纬度到 ENU 转换
- 集群级任务语义映射
- 6 机编队控制与搜索执行

也就是说，QGC 现在已经可以作为你的集群任务上位机来用。