# 部署侧安全阈值标定 — 训练侧交接文档

> 交接方：部署侧（rl_real_JXG 真机控制）　承接方：策略训练
> 日期：2026-10-08　状态：待承接
> 模型：`policy/JXGw/himloco/policy_v24_deploybias_m3400_sim.onnx`（当前部署版）

---

## 1. 背景：为什么需要你们

真机部署侧计划新增三道"防暴走"保护，阈值必须基于**策略的正常行为分布**来定：

| 保护 | 现状 | 问题 |
|---|---|---|
| 输出幅值 clip | `clip_actions = ±100` | 换算关节空间 ±25rad，**形同虚设** |
| 动作变化率限制 | 无 | 振荡/发散型暴走无拦截 |
| 关节目标硬包络 | 无 | 关节目标可越过物理限位 |

这个分布真机采集危险且低效；**sim-to-sim（mujoco 加载同一个 onnx）是最高效安全的数据来源**，这正是需要训练侧的原因——你们对 sim 环境和策略行为最熟悉。

## 2. 需要你们做的事（总览）

1. 用 mujoco sim-to-sim 跑 §4 的工况清单，录制 action 序列
2. 按 §6 的统计方法算出分位数
3. 按 §7 的格式回填 `safety_limits.yaml` 交回部署侧

预计工作量：环境搭建半天 + 采集 1 小时 + 统计半小时。

## 3. 环境

- **模型**：`policy/JXGw/himloco/policy_v24_deploybias_m3400_sim.onnx`
- **配置**：`policy/JXGw/himloco/config.yaml`（obs=57、history=6、action_scale=0.25、
  default_dof_pos、rl_kp/kd、torque_limits [60,60,120,17] 均以此为准）
- **sim-to-sim 构建**：仓库根 `bash build.sh` 选 mujoco（首次先跑 `scripts/download_mujoco.sh`）
- **控制频率**：50Hz（与部署推理频率一致，变化率阈值才可直接换算）

## 4. 采集工况清单（每个 ≥60s）

| # | 工况 | 指令 (vx, vy, yaw) | 备注 |
|---|---|---|---|
| 1 | 原地站立 | (0,0,0) | 基线，含进入后前 5s |
| 2 | 低速前进 | (0.5,0,0) | |
| 3 | 中速前进 | (1.0,0,0) | |
| 4 | 高速前进 | (1.5,0,0) | 若训练域内最高速再取 |
| 5 | 原地转向 | (0,0,±1.0) | 两方向都要 |
| 6 | 侧移 | (0,±0.5,0) | vy 使能后 |
| 7 | 行进急停 | (1.0,0,0)→(0,0,0) | vel_stop，重点录突变段 |
| 8 | 完整流程 | 趴→起→走→趴 | 进出 RL 的过渡段 |

## 5. 录制内容（每帧一行，npz 或 csv）

```
t, action[16], output_dof_pos[16], dof_pos[16], dof_vel[16], cmd(vx,vy,yaw)
```

- `action`：策略网络原始输出（clip 前），**这是核心**
- `output_dof_pos`：拼装后的关节目标（default_dof_pos + 0.25×action）
- 分轮腿统计：轮关节 idx [3,7,11,15] 是速度模式（kp=0/kd=1.0），动力学与腿不同

## 6. 统计方法（可直接跑）

```python
import numpy as np
# a: (N, 16) action 序列，leg_idx=除[3,7,11,15]外的12列，wheel_idx=[3,7,11,15]
leg, wheel = [i for i in range(16) if i not in (3,7,11,15)], [3,7,11,15]
da_leg   = np.abs(np.diff(a[:, leg],   axis=0))   # 腿:逐帧差分绝对值
da_wheel = np.abs(np.diff(a[:, wheel], axis=0))
print("Δaction 腿  p95/p99/p99.9/max:", np.percentile(da_leg, [95,99,99.9]), da_leg.max())
print("Δaction 轮  p95/p99/p99.9/max:", np.percentile(da_wheel,[95,99,99.9]), da_wheel.max())
print("|action| 腿 max/p99.99:", np.abs(a[:,leg]).max(), np.percentile(np.abs(a[:,leg]),99.99))
```

**阈值规则**（裕量系数可按分布尾部形状调整）：

- `clip_actions`（新）= |action|max × 2　→ 正常行为零触界，仅在暴走发散时兜底
- `action_rate_limit` = Δaction_p99.9 × 1.5（腿/轮分开定）
- `joint_envelope`（指令侧）= default_dof_pos ± action_scale×|action|max×1.5，
  **再与 URDF 物理限位内缩 0.15rad 取交集**（部署侧已有物理限位数据）

## 7. 交付格式：`safety_limits.yaml`

```yaml
JXGw/himloco:
  # 采集自 sim-to-sim v24, 统计样本 N=?, 日期=?
  clip_actions: ±?.?            # 关节空间: ±(action_scale × 上表值)
  action_rate_limit_leg:  ?.??  # rad/帧 @50Hz, 12 腿关节
  action_rate_limit_wheel: ?.?? # rad/帧 @50Hz, 4 轮关节
  joint_envelope_lower: [?×16]  # 与 URDF 内缩限位取交集后
  joint_envelope_upper: [?×16]
  notes: "急停段 p99.9 异常偏大,已/未单独剔除(说明处理方式)"
```

## 8. 给训练侧的两个说明

1. **限幅与 obs 的一致性**：obs 里含上帧 `actions`（57 维观测的一部分）。部署侧加
   rate-limit 后，"喂给 obs 的 action"将与"执行到电机的 action"保持一致（都取限幅后的值），
   维持训练时"执行即反馈"的语义——不会造成分布漂移，但阈值若过紧会削步态响应，所以取
   p99.9×1.5 而非 p50。
2. **下次训练的建议**（可选，不阻塞本次交付）：训练时对 action 加 rate 限制作为
   observation noise / action delay 同级的 domain randomization，sim-to-real 差距会更小。

## 9. 部署侧分工（我方，供知悉）

- 读取 `safety_limits.yaml`，在控制环实现 clip/rate/envelope 三道检查 + 倾覆/IMU/过温/NaN 看门狗
- 阈值生效顺序：NaN 拦截 → clip → rate → envelope → （反馈侧）物理限位 → SafeHold
- 真机试运行后回传统计量（超限触发次数），若某阈值频繁触发则反馈你们复核分布

## 10. 待确认项

- [ ] 训练域内 vx/vy/yaw 的上限值（§4 工况 3-6 按此调整）
- [ ] v24 之外是否需要给旧模型（v3~v17）也出一份标定（部署只跑 v24，默认不需要）
- [ ] 交付时间与回传路径
