# Change Log
## 2025.06.10
新增避障
## 2025.06.08
Chase1是一个优化版本的追球节点，相比原始版本有以下主要改进：
### 1. 球的运动预测与拦截
- 添加了球速度计算功能，通过连续位置计算球的速度
- 当球速大于0.5m/s时自动启用拦截模式
- 在拦截模式下，机器人会提前移动到预测位置
```cpp
// 在GameObject中添加速度相关字段
struct GameObject {
    // ... 其他字段 ...
    Point lastPosToField;            // 上一次在场地坐标系中的位置
    rclcpp::Time lastTimePoint;      // 上一次更新的时间
    Point velocityToField;           // 在场地坐标系中的速度 (m/s)
};

// 在detectProcessBalls中计算速度
if (data->ball.lastTimePoint.nanoseconds() != 0) {
    double dt = (data->ball.timePoint - data->ball.lastTimePoint).seconds();
    if (dt > 0 && dt < 1.0) {
        data->ball.velocityToField.x = (data->ball.posToField.x - data->ball.lastPosToField.x) / dt;
        data->ball.velocityToField.y = (data->ball.posToField.y - data->ball.lastPosToField.y) / dt;
    }
}
```

### 2. 智能绕球策略
- 基于机器人到球的角度动态选择最优绕球方向
- 避免不必要的方向切换
- 优化了绕球路径的计算
```cpp
double robotToBallAngle = atan2(
    brain->data->ball.posToField.y - brain->data->robotPoseToField.y,
    brain->data->ball.posToField.x - brain->data->robotPoseToField.x
);
```

### 3. 改进的速度控制
- 引入了基于距离和角度的双重速度因子
- 距离因子：随距离增加而增加速度
- 角度因子：当偏离目标角度较大时降低速度
```cpp
double distFactor = 1.0 / (1.0 + exp(-2 * (ballRange - 1.5))); // 距离因子
double angleFactor = 1.0 / (1.0 + exp(5 * (fabs(ballYaw) - 0.5))); // 角度因子
double speedFactor = distFactor * angleFactor;
```

### 4. 平滑运动控制
- 添加了速度平滑处理
- 使用alpha滤波器避免速度突变
- 提供更流畅的运动体验
```cpp
const double alpha = 0.7; // 平滑因子
vx = alpha * vx + (1 - alpha) * last_vx;
vy = alpha * vy + (1 - alpha) * last_vy;
vtheta = alpha * vtheta + (1 - alpha) * last_vtheta;
```

### 5. 调试功能
- 添加了详细的状态输出
- 实时显示关键参数
- 便于调试和参数调整
```cpp
brain->log->logToScreen("Chase1",
    format("State: %s, BallRange: %.2f, BallSpeed: %.2f, SpeedFactor: %.2f",
        _state.c_str(), ballRange, ballSpeed, speedFactor),
    0x00FF00FF);
```

### 使用方法

在行为树XML文件中添加Chase1节点：
```xml
<Chase1 
    vx_limit="1.0"      <!-- 前进速度限制 -->
    vy_limit="0.4"      <!-- 横向速度限制 -->
    vtheta_limit="1.0"  <!-- 旋转速度限制 -->
    dist="1.0"          <!-- 与球保持的距离 -->
/>
```

### 可调参数

1. 预测相关
   - predictTime: 预测时间（当前0.5秒）
   - ballSpeed阈值: 拦截模式触发速度（当前0.5m/s）

2. 速度控制
   - distFactor参数: -2和1.5
   - angleFactor参数: 5和0.5
   - alpha: 速度平滑因子（当前0.7）

3. 距离控制
   - dist: 与球保持的距离
   - chase状态切换阈值: 1.0

### 注意事项

1. 速度限制参数(vx_limit, vy_limit, vtheta_limit)应根据机器人实际性能设置
2. 平滑因子alpha的值(0.7)可以根据实际运动效果调整
3. 预测时间可根据实际场景调整，较大的值可能导致预测不准确
4. 球速度计算依赖于连续的位置信息，可能会有一定的延迟和噪声

## 最近更新
- 2025-06-08: 优化了球速度计算方法，将速度计算逻辑移至detectProcessBalls函数中，提高了速度计算的准确性
- 2025-06-10: 优化了TurnOnSpot节点的旋转速度，提高了机器人旋转找球的效率

## RobotFindBall1节点改进说明

RobotFindBall1是一个优化版本的球查找节点，相比原始版本有以下主要改进：

### 1. 智能方向选择
- 基于球的最后已知位置决定搜索方向
- 当没有球的历史信息时，随机选择方向
```cpp
if (useLastBallPos && brain->data->ball.lastSeenTime.seconds() > 0) {
    // 如果最近看到过球，根据最后已知的球位置决定搜索方向
    _turnDir = brain->data->ball.yawToRobot > 0 ? 1.0 : -1.0;
} else {
    // 否则随机选择一个方向
    _turnDir = (brain->get_clock()->now().seconds() * 10) - 
               int(brain->get_clock()->now().seconds() * 10) > 0.5 ? 1.0 : -1.0;
}
```

### 2. 动态方向切换
- 定期改变搜索方向，避免一直在同一区域搜索
- 可配置的搜索时间参数
```cpp
// 如果搜索时间超过设定值，改变搜索方向
if (timeSinceLastDirectionChange > scanTime) {
    _turnDir = -_turnDir; // 反转方向
    _lastDirectionChangeTime = currentTime;
}
```

### 3. 自适应转速
- 根据搜索时间动态调整转速
- 长时间未找到球时增加转速
```cpp
// 根据搜索时间动态调整转速
double speedFactor = 1.0;
if (elapsedTime > scanTime * 2) {
    // 长时间未找到球，增加转速
    speedFactor = 1.2;
}
```

### 4. 详细的状态日志
- 记录搜索状态和方向变化
- 便于调试和问题排查
```cpp
brain->log->logToScreen("RobotFindBall1", 
    format("Searching for ball: %.1f seconds, dir: %.1f, speed: %.2f", 
           elapsedTime, _turnDir, vyawLimit * speedFactor),
    0x0000FFFF);
```

### 使用方法

在行为树XML文件中添加RobotFindBall1节点：
```xml
<RobotFindBall1 
    vyaw_limit="2.0"           <!-- 旋转速度限制 -->
    scan_time="5.0"            <!-- 每次搜索方向的时间 -->
    use_last_ball_pos="true"   <!-- 是否使用球的最后已知位置 -->
/>
```

### 可调参数

1. 旋转相关
   - vyaw_limit: 旋转速度限制（推荐1.5-2.0）
   - scan_time: 每个方向搜索时间（推荐5-10秒）
   - speedFactor: 长时间搜索的速度增益（当前1.2）

2. 方向选择
   - use_last_ball_pos: 是否使用球的历史信息

## TurnOnSpot节点优化说明

### 优化目标
提高TurnOnSpot节点的旋转速度，使机器人在搜索球和调整姿态时能够更快完成旋转动作，提高整体效率。

### 主要改进
1. **动态速度调整**
   - 基础速度系数从2.0提高到2.5
   - 根据剩余旋转角度动态调整速度系数：
     - 当剩余角度大于1.0弧度时，使用更高的3.0速度系数
     - 接近目标角度时自动降低速度，避免过冲

2. **速度限制机制**
   - 设置最小旋转速度为1.5 rad/s，确保转向不会太慢
   - 设置最大旋转速度为3.0 rad/s，保证旋转稳定性
   - 根据剩余角度自动调整实际速度值

3. **增强日志记录**
   - 添加详细的屏幕日志输出，显示剩余角度和当前速度
   - 方便调试和参数调优

### 使用示例
在行为树中使用TurnOnSpot节点时，无需修改参数，节点内部已实现速度优化：

```xml
<TurnOnSpot rad="3.14" towards_ball="true" />
```

### 性能提升
- 完成180度旋转所需时间显著减少
- 保持了旋转精度，避免了过冲问题
- 在StrikerFindBall行为树中提高了找球效率

## StrikerFindBall行为树说明

StrikerFindBall是一个专为前锋设计的多阶段找球策略，结合了快速旋转和精确搜索：

### 1. 多阶段搜索策略
- 第一阶段：使用RobotFindBall1快速旋转寻找球
- 第二阶段：如果未找到，使用TurnOnSpot精确旋转180度
- 第三阶段：如果仍未找到，移动到更好的位置继续搜索

### 2. 行为树结构
```xml
<BehaviorTree ID="StrikerFindBall">
   <Sequence name="root">
      <!-- 确保机器人在场地内 -->
      <ReactiveSequence>
          <GoBackInField valve="0.3"/>
      </ReactiveSequence>
      
      <!-- 先快速旋转找球 -->
      <RobotFindBall1 vyaw_limit="2.0" scan_time="5.0" use_last_ball_pos="true" />
      
      <!-- 如果快速旋转没找到，尝试精确的180度转向 -->
      <Sequence _while="!ball_location_known">
        <TurnOnSpot rad="3.14" towards_ball="true"/>
      </Sequence>
      
      <!-- 如果还是没找到，移动到更好的位置继续找 -->
      <Sequence _while="!ball_location_known">
        <MoveToPoseOnField x="-2.0" vtheta_limit="2.0" vx_limit="0.4" />
        <RobotFindBall1 vyaw_limit="1.8" scan_time="6.0" use_last_ball_pos="true" />
      </Sequence>
    </Sequence>
</BehaviorTree>
```

### 3. 与原有策略的集成
在striker_play.xml中使用Fallback节点组合新旧找球行为：
```xml
<Fallback _while="decision=='find'">
   <!-- 首先尝试新的原地旋转找球行为 -->
   <SubTree ID="StrikerFindBall" _autoremap="true" />
   <!-- 如果新方法失败，回退到原有的找球策略 -->
   <SubTree ID="FindBall" _autoremap="true" />
</Fallback>
```

### 优势
1. 快速响应：立即开始原地旋转，不浪费时间
2. 高效搜索：使用更高的旋转速度和智能方向选择
3. 分阶段策略：从快速到精确，逐步扩大搜索范围
4. 兼容性好：保留原有找球行为作为备份，确保稳定性

### 使用场景
- 前锋在比赛中丢失球时
- 需要快速找回球的位置
- 需要在有限时间内最大化找球成功率

### 注意事项
1. vyaw_limit参数应根据机器人实际性能设置
2. scan_time参数影响方向切换频率，可根据场地大小调整
3. MoveToPoseOnField的目标位置可根据战术需求调整

## 2025.05.30
1. Add Subtree find ball, which is a demo of how to find ball with moving, fix the problem of if not find ball, robot
will not move
## 2025.05.29
1. Change joystick subscription from /joy to /remote_controller_state, which is internal msg of Booster controller,
as /remote_controller_state handle different type of joystick internally, which means which type of joystick is
transparent to the brain_node, and thus no more need for joystick type.