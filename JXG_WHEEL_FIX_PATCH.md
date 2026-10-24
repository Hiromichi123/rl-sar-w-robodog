# JXG 轮速失控热修复方案（方案 A）

## 问题描述
策略在 `cmd_vel=[0,0,0]` 时仍输出 ±8~19 rad/s 轮速动作，导致轮足电机"疯转"。

## 根因分析
1. **训练域随机化副作用**：push robot + 噪声导致零速度保持能力差
2. **cmd_vel 噪声放大**：`commands_scale=[2.0,2.0,0.25]` 将手柄漂移放大 2 倍
3. **无动作平滑**：轮速目标直接输出，无低通滤波

## 修复方案（三处修改）

### 修改 1：cmd_vel 死区（消除手柄漂移）

**文件**：`src/rl_sar/src/rl_real_JXG.cpp`
**位置**：`CmdvelCallback` 函数（~line 1080）

```cpp
void RL_Real::CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->last_cmd_vel_time_ = std::chrono::steady_clock::now();

    // 【新增】cmd_vel 死区：绝对值 < 0.05 视为 0
    auto deadzone = [](float v) { return std::abs(v) < 0.05f ? 0.0f : v; };
    msg->linear.x  = deadzone(msg->linear.x);
    msg->linear.y  = deadzone(msg->linear.y);
    msg->angular.z = deadzone(msg->angular.z);

    this->cmd_vel = *msg;

    // 原有代码保持不变...
    if (this->control.GetGamepadOverride()) {
        PublishRosFeedback("cmd_vel ignored: gamepad override active");
    }
}
```

### 修改 2：轮速目标低通滤波（消除高频抖动）

**文件**：`src/rl_sar/src/rl_real_JXG.cpp`
**位置**：`ModelInference` 函数，`ComputeOutput` 调用之后（~line 878）

```cpp
this->obs.actions = this->Forward();
this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

// 【新增】轮速目标低通滤波：alpha=0.3
static std::vector<float> prev_wheel_vel(16, 0.0f);
const float alpha = 0.3f;
for (int i : this->params.Get<std::vector<int>>("wheel_indices")) {
    float raw = this->output_dof_vel[i];
    this->output_dof_vel[i] = alpha * raw + (1.0f - alpha) * prev_wheel_vel[i];
    prev_wheel_vel[i] = this->output_dof_vel[i];
}

this->PublishAction();
// 原有代码保持不变...
```

### 修改 3：cmd=0 时强制轮速目标=0（激进，可选）

**文件**：`src/rl_sar/src/rl_real_JXG.cpp`
**位置**：同修改 2，在低通滤波**之后**追加

```cpp
// 【新增】cmd=0 时强制轮速目标=0
bool cmd_zero = (std::abs(this->cmd_vel.linear.x) < 0.01f &&
                 std::abs(this->cmd_vel.linear.y) < 0.01f &&
                 std::abs(this->cmd_vel.angular.z) < 0.01f);
if (cmd_zero) {
    for (int i : this->params.Get<std::vector<int>>("wheel_indices")) {
        this->output_dof_vel[i] = 0.0f;
    }
}
```

**注意**：修改 3 会完全禁止零命令时的轮速微调，可能影响平衡。建议先只改 1+2，观察效果后再决定是否加 3。

## 编译与部署

```bash
cd /home/jinjiao/rl_sar-w2_recovered_v4
colcon build --packages-select rl_sar
# 重启策略进程
```

## 验证方法

1. 启动后保持手柄静止，观察轮速是否降至 <1 rad/s
2. 轻推手柄给微小 cmd，确认死区外响应正常
3. 快速摇动手柄，确认轮速无高频抖动

## 回滚

```bash
cd /home/jinjiao/rl_sar-w2_recovered_v4
git checkout src/rl_sar/src/rl_real_JXG.cpp
colcon build --packages-select rl_sar
```

## 备注

- 本方案为**热修复**，治标；根治需重新训练（已启动 `jxg_him_dev_v2`，含 stop-hold 课程）
- 训练完成后建议部署新模型并移除此补丁（或保留修改 1 的死区作为安全冗余）
