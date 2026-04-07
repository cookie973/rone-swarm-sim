# QGC 集群任务简明手册

这份手册只说明一件事：在 QGroundControl 地图上怎么下任务，才能驱动你现在这套集群控制器。

## 运行前提

- `qgc_bridge` 已启动。
- `formation_commander` 已启动。
- 至少已经收到一架参考机的 `/uav1/uav/navsatfix`，这样桥接层才能把 QGC 经纬度转换成集群 ENU 坐标。

## 已验证映射

以下映射已经做过在线验证：

- 单个 `WAYPOINT`：会发布到 `/swarm/mission_center`
- 多个 `WAYPOINT`：会发布到 `/swarm/search_mission`
- `LOITER_UNLIM`：会发布到 `/swarm/search_orbit`
- 单个 `RETURN_TO_LAUNCH`：会发布到 `/swarm/formation = origin_land`

## 地图操作规则

### 1. 编队整体飞到某个点

在 QGC Mission 里只放 1 个 `Waypoint`。

效果：

- 桥接为 `/swarm/mission_center`
- 编队中心整体飞向该点
- 当前编队形状保持不变

适用场景：

- 编队前往目标区域
- 编队保持阵型平移

### 2. 多目标搜索

在 QGC Mission 里连续放多个 `Waypoint`。

效果：

- 桥接为 `/swarm/search_mission`
- 每个点都会变成一个搜索目标
- 集群控制器按权重分配无人机数量

参数约定：

- `param1`：该搜索点权重
- 不填或填 `<= 0`：自动回退为默认权重 `1.0`

建议：

- 权重越大，该点分到的无人机越多
- 搜索点数量必须小于无人机数量
- 如果你有 6 架机，建议搜索点最多 5 个

例子：

- 3 个点，`param1 = 3.0 / 1.5 / 0.8`
- 会被桥接成 3 个带权重的 `/swarm/search_mission` 目标

### 3. 单点盘旋搜索

在 QGC Mission 里放一个 `Loiter Unlimited`。

效果：

- 桥接为 `/swarm/search_orbit`
- 编队先飞到盘旋中心附近，再进入盘旋搜索

参数约定：

- `param3`：盘旋半径，单位米
- 不填：自动回退为默认半径 `12.0`

补充：

- 当前默认角速度是 `0.12 rad/s`
- 如果 `Loiter` 本身没有坐标，桥接层会尝试取前一个任务点坐标作为盘旋中心

### 4. 原点返航降落

在 QGC Mission 里只放 1 个 `Return to Launch`。

效果：

- 桥接为 `/swarm/formation = origin_land`
- 集群执行回原点并降落

注意：

- 这个映射要求 Mission 中只有这一个 RTL 项
- 如果你把 RTL 混在其他任务项里，当前桥接不会把它当成独立“返原点降落任务”处理

### 5. 清空当前任务

在 QGC 侧执行清空 Mission。

效果：

- 桥接为 `/swarm/clear_task`
- 当前集群任务会被清掉

## 当前不建议的画法

- 不要把大量不同类型命令混在一个 Mission 里，尤其是 `Waypoint + Loiter + RTL + 其他命令` 的复杂串联。
- 当前桥接设计是“把 QGC 任务当作集群意图输入”，不是“逐项替你跑完整单机航线脚本”。

更稳的做法：

- 一个 Mission 只表达一种集群意图
- 例如：
  - 只放 1 个 `Waypoint`
  - 或只放多个 `Waypoint`
  - 或只放 1 个 `Loiter`
  - 或只放 1 个 `RTL`

## 常见现象

### Mission 已上传成功，但 `/swarm/status` 里目标还是空的

这通常不是桥接没工作，而是控制器当前处在地面安全态。

当前控制器逻辑是：

- 如果无人机没有整体进入可执行任务的飞行状态
- 某些任务会被立即清掉，避免地面态残留旧任务

所以要区分两件事：

- 桥接层有没有正确发布 `/swarm/mission_center`、`/swarm/search_mission`、`/swarm/search_orbit`、`/swarm/formation`
- 控制器在当前飞行状态下是否决定保留并执行这个任务

## 建议操作顺序

1. 先启动 SITL、`mavlink_tx`、`formation_commander`、`qgc_bridge`
2. 确认 `/uav1/uav/navsatfix` 正常
3. 用现有起飞流程让集群进入飞行状态
4. 在 QGC 地图上只下一个明确意图的 Mission
5. 观察 `/swarm/status` 和飞机行为

## 目前最适合展示的四种操作

1. 1 个 `Waypoint`：展示编队整体机动
2. 3 个 `Waypoint`：展示多目标搜索分配
3. 1 个 `Loiter Unlimited`：展示单点盘旋搜索
4. 1 个 `Return to Launch`：展示返原点降落