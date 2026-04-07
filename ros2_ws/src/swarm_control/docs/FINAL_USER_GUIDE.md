# 6 机蜂群系统最终使用说明

这份说明只覆盖当前已经实现、已经联通、并且日常可用的功能，不再依赖 QGroundControl。

## 一、系统组成

当前系统由三部分组成：

1. 仿真与飞控链路
   - Gazebo Harmonic
   - 6 个 ArduPilot SITL
   - 6 个 `mavlink_tx`

2. 集群控制器
   - `formation_commander`
   - 负责队形切换、任务规划、搜索模式、返原点降落

3. 上位机
   - `swarm_ground_station`
   - 负责一键起飞、降落、任务下发、状态显示

## 二、当前可用功能

1. 一键起飞
2. 当前点位起飞
3. 当前点位降落
4. 返回初始位置降落
5. 清空当前任务
6. 编队飞行前往目标点
7. 静止状态编队切换
8. 飞行中编队切换
9. 搜寻模式 1：多目标点加权搜索
10. 搜寻模式 2：单点盘旋搜索

## 三、推荐启动顺序

### 1. 启动仿真全栈

在终端执行：

```bash
cd /home/cookie/gz_ws/src/ardupilot_gazebo/scripts
bash swarm_stop.sh
bash swarm_start.sh --no-controller
```

说明：

- `swarm_start.sh` 会启动 Gazebo、6 个 SITL、6 个 bridge，以及基础链路
- 使用 `--no-controller`，是为了把集群控制器独立交给 ROS 2 启动，便于调试和重启

### 2. 启动集群控制器

在终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/cookie/ros2_ws/install/setup.bash
ros2 launch swarm_control formation_cmd.launch.py
```

### 3. 启动上位机

在另一个终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/cookie/ros2_ws/install/setup.bash
ros2 run swarm_control swarm_ground_station
```

## 四、推荐起飞方式

### 方式 A：直接用上位机一键起飞

在上位机中：

1. 填写 `Takeoff Alt`
2. 点击 `一键起飞`

适用场景：

- 正常联调
- 论文演示
- 地面站完整功能展示

### 方式 B：脚本起飞（更适合排故）

如果上位机起飞没有响应，使用脚本：

```bash
cd /home/cookie/gz_ws/src/ardupilot_gazebo/scripts
TAKEOFF_STOP_CONTROLLER=0 TAKEOFF_ENGINE=cli TAKEOFF_PARALLEL=1 TAKEOFF_ARM_SERIAL_FIRST=1 TAKEOFF_RETRY_ROUNDS=4 bash ./takeoff_all.sh 5.0
```

说明：

- 这是当前验证过的稳定起飞路径
- 目标高度可替换 `5.0`

## 五、上位机功能怎么用

### 1. 编队飞行前往目标点

在 `Mission Center` 区域输入：

- `Target X (East)`
- `Target Y (North)`

然后点击：

- `编队飞行前往目标点`

效果：

- 编队中心整体移动
- 当前队形保持不变

### 2. 编队切换

在 `编队切换（静止/飞行中）` 区域点击相应队形按钮。

当前支持的主要队形：

- `line_x`
- `line_y`
- `t_shape`
- `echelon`
- `rectangle`
- `v_shape`
- `origin_grid`

说明：

- 静止时可切换
- 飞行中也可切换
- 飞行中切换时控制器会自动进行分层避碰与重构

### 3. 当前点位降落

点击：

- `当前点位降落`

效果：

- 先清空当前任务
- 再下发降落命令

### 4. 返回初始位置降落

点击：

- `返回初始位置降落`

效果：

- 编队先回原始编队中心
- 然后统一降落

### 5. 清空当前任务

点击：

- `清空当前任务`

效果：

- 清除当前 mission / search / orbit / steady target

### 6. 搜寻模式 1：多目标点加权搜索

在 `搜寻模式1：多目标点加权搜索` 区域逐行填写：

- 目标 ID
- X
- Y
- Weight

然后点击：

- `启动搜寻模式1`

说明：

- 权重越大，分配到该点的无人机数量越多
- 目标点数量必须小于无人机数量

### 7. 搜寻模式 2：单点盘旋搜索

在 `搜寻模式2：单点盘旋搜索` 区域填写：

- `Center X`
- `Center Y`
- `Radius`
- `Angular Speed`

然后点击：

- `启动搜寻模式2`

效果：

- 集群先飞向目标中心
- 然后进入单点盘旋搜索

## 六、推荐日常流程

1. 启动仿真全栈
2. 启动 `formation_commander`
3. 启动 `swarm_ground_station`
4. 一键起飞到 5 m
5. 选择一个任务：
   - 编队前往目标点
   - 编队切换
   - 搜寻模式 1
   - 搜寻模式 2
6. 任务结束后：
   - 当前点位降落，或
   - 返回初始位置降落

## 七、关闭顺序

### 1. 停止上位机和控制器

直接关闭对应终端，或 `Ctrl+C`

### 2. 停止仿真全栈

在终端执行：

```bash
cd /home/cookie/gz_ws/src/ardupilot_gazebo/scripts
bash swarm_stop.sh
```

## 八、常见现象

### 1. 点击按钮没反应

优先检查：

1. `formation_commander` 是否在线
2. `/swarm/status` 是否有发布者
3. 6 架机的 `/uavX/uav/odom`、`/uavX/uav/navsatfix` 是否存在

### 2. 起飞后出现旧任务残留

先点击：

- `清空当前任务`

再进行新任务操作。

### 3. 地面状态下任务不保持

这是当前控制器的安全策略：

- 如果无人机没有整体进入可执行任务的飞行状态
- 某些任务会被立即清掉

所以建议始终先起飞，再执行任务。

## 九、推荐展示功能

如果要做演示，最稳的四项是：

1. 一键起飞
2. 编队飞往目标点
3. 飞行中编队切换
4. 单点盘旋搜索或多目标加权搜索

最后用：

5. 返回初始位置降落
