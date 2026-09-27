# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class JXGRoughCfg(LeggedRobotCfg):
    # 训练环境
    class env(LeggedRobotCfg.env):
        num_envs = 4096
        num_one_step_observations = 3 + 3 + 3 + 16 + 16 + 16
        num_observations = num_one_step_observations * 6
        num_one_step_privileged_obs = num_one_step_observations + 3 + 3 + 11 * 17 + 12
        num_privileged_obs = num_one_step_privileged_obs * 1
        num_actions = 16
        env_origin_jitter = 1.0

    # 地形
    class terrain(LeggedRobotCfg.terrain):
        mesh_type = 'plane'
        curriculum = False  # plane 网格不支持地形课程
        static_friction  = 0.8
        dynamic_friction = 0.8
        max_init_terrain_level = 0
        terrain_proportions = [1.0, 0.0, 0.0, 0.0, 0.0]

    # 速度指令 — Route 1 v3 (correct semantics!): ONLY lift MAX_CURRICULUM ceiling so curriculum RAMP works.
    # IMPORTANT: ranges.lin_vel_x KEPT at [-1.0,1.0] (initial sampling range — matches Phase1 policy's OOD-free dist).
    # Curriculum then RAISES lin_vel_x ±0.2 per episode once tracking_lin_vel > 80% max, UP TO max_curriculum=2.0.
    # v5: resampling_time 10→5 (double the stop/reverse transients per episode — fix lateral sway on sudden stops)
    class commands(LeggedRobotCfg.commands):
        curriculum = True
        max_curriculum = 2.0  # was 1.5 — new vx ceiling after curriculum ramp saturates (target 2.0 m/s)
        num_commands = 4
        resampling_time = 5.
        heading_command = True
        class ranges:
            lin_vel_x = [-1.0, 1.0]  # UNCHANGED from Phase1! initial sampling range (critical for stable first buckets)
            lin_vel_y = [-0.6, 0.6]
            ang_vel_yaw = [-1.0, 1.0]
            heading = [-3.14, 3.14]

    # 初始状态
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.45]
        default_joint_angles = {
            'FL_hip_joint': 0.0,
            'FR_hip_joint': 0.0,
            'RL_hip_joint': 0.0,
            'RR_hip_joint': 0.0,
            'FL_thigh_joint': -0.55,
            'FR_thigh_joint': -0.55,
            'RL_thigh_joint': 0.55,
            'RR_thigh_joint': 0.55,
            'FL_calf_joint': 1.68,
            'FR_calf_joint': 1.68,
            'RL_calf_joint': -1.68,
            'RR_calf_joint': -1.68,
            'FL_foot_joint': 0.0,
            'FR_foot_joint': 0.0,
            'RL_foot_joint': 0.0,
            'RR_foot_joint': 0.0,
        }

    # PD控制
    class control( LeggedRobotCfg.control ):
        control_type = 'P'
        stiffness = {'hip_joint': 200.,'thigh_joint': 200.,'calf_joint': 200.,"foot_joint":0}
        damping =   {'hip_joint': 5,'thigh_joint': 5,'calf_joint': 5,"foot_joint":1.0}
        action_scale = 0.25
        vel_scale = 10.0
        decimation = 4
        wheel_speed = 1

    # URDF资源
    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/JXG/urdf/JXG.urdf'
        name = "JXG"
        foot_name = "foot"
        wheel_name =["foot"]
        penalize_contacts_on = ["thigh", "calf", "base"]
        terminate_after_contacts_on = ['base']
        priviledge_contacts_on = ["thigh", "calf", "base"]
        self_collisions = 1
        replace_cylinder_with_capsule = False
        flip_visual_attachments = False

    # 奖励函数：简化版，先学会走路
    #  --- PHASE 1 adjustment (2026-08-26 baseline 2000 iter resume) ---
    #  - turn reward slightly up (encourage stepping turn not wheel scuff)
    #  - run_still penalty x10 (stop "commanded vel but wheel-scuff only" behaviour)
    #  --- v5 stop-sway fix (2026-09-05): lateral sway on sudden stop/reverse (Roll ±0.3-0.5° limit cycle)
    #  - action_rate -0.01→-0.05 (5x: penalize high-freq action oscillation)
    #  - ang_vel_xy  -0.05→-0.20 (4x: penalize body roll/pitch angular velocity jitter)
    # 域随机化 — v6 sliprand (2026-09-06): 真机停车左右摇晃残余（v5+kp200 仍有 ±3~4 rad/s 停车偏航踢）
    #  - 每轮独立摩擦因子 [0.6,1.4]×基准：左右/前后不对称，策略必须用观测反馈而非开环差速纠偏
    #  - 轮速观测噪声 4.0（默认1.5）：模拟打滑造成的轮速度量误差
    #  - 停车保持课程：30% env 零指令保持 2~6s（期间 push 生效），专训停车稳态
    class domain_rand(LeggedRobotCfg.domain_rand):
        wheel_friction_asym = True
        wheel_friction_factor_range = [0.70, 1.30]
        stop_hold_enable = True
        stop_hold_prob = 0.3
        stop_hold_duration_range = [2.0, 6.0]

    class noise(LeggedRobotCfg.noise):
        class noise_scales(LeggedRobotCfg.noise.noise_scales):
            wheel_dof_vel = 4.0

    class rewards( LeggedRobotCfg.rewards ):
        class scales:
            tracking_lin_vel = 1.5
            tracking_ang_vel = 1.5  # +0.3 → stronger steering tracking reward (was 1.2)

            lin_vel_z = -1.0
            ang_vel_xy = -0.25
            orientation = -0.5

            base_height = -10.0
            hip_default = -0.5
            stand_still = -0.5

            collision = -1.0
            feet_stumble = -0.1

            # 平滑：降低action_rate让策略能学出动作
            action_rate = -0.05
            torques = -5e-4
            dof_vel = -1e-7
            dof_acc = -1e-7

            run_still = -0.50  # x10 from -0.05; strongly penalise "have vel cmd but no stepping motion"

        only_positive_rewards = False
        tracking_sigma = 0.25
        soft_dof_pos_limit = 1.
        soft_dof_vel_limit = 1.
        soft_torque_limit = 1.
        base_height_target = 0.40
        max_contact_force = 320.


class JXGRoughCfgPPO( LeggedRobotCfgPPO ):
    class policy(LeggedRobotCfgPPO.policy):
        init_noise_std = 0.5

    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01

    class runner( LeggedRobotCfgPPO.runner ):
        save_interval = 500   # checkpoint every 500 iter (phase1 proved value — keep)
        num_steps_per_env = 48
        max_iterations = 2500 # v6: resume 10000 → train 2500 MORE (runner 语义是增量: 10000+2500=12500 终点)
        experiment_name = 'JXG'
        # --- v6 sliprand (resume from v5_stopsway final model_10000, 2026-09-06) ---
        run_name = "studio_test_tag"
        # Resume from v5_stopsway final model_10000 (2026-09-05).
        # Plan A v3 resume fix (task_registry.py L152-163) will PREFER cfg.runner.resume_path when absolute.
        resume = True
        resume_path = '/home/bygpu/Desktop/HIMLoco-for-Go2W-main/HIMLoco-for-Go2W-main/logs/JXG/Sep05_13-02-59_walk_flat_v5_stopsway/model_10000.pt'
        load_run = -1
        checkpoint = -1
