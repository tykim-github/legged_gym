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

class HopperCfg( LeggedRobotCfg ):
    class env( LeggedRobotCfg.env):
        num_envs = 512
        num_observations = 169
        num_actions = 9
        env_spacing = 3.  # not used with heightfields/trimeshes 
        send_timeouts = True # send time out information to the algorithm
        episode_length_s = 20 # episode length in seconds

    
    class terrain( LeggedRobotCfg.terrain):
        mesh_type = 'plane' # "heightfield" # none, plane, heightfield or trimesh
        measure_heights = False 
        
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 1.] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            "hipjoint"  : 0., 
            "kneejoint" : 0., 
            "anklejoint": 0.
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        stiffness = {'hipjoint': 10., 'kneejoint': 10., 'anklejoint': 10.}  # [N*m/rad]
        damping = {'hipjoint': 1.0, 'kneejoint': 1.0, 'anklejoint': 1.0}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.5
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4
        
    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/hopper/urdf/hoprun.urdf'
        name = "hopper"
        foot_name = 'foot'
        penalize_contacts_on = ["pelvis", "femur", "tibia"]
        terminate_after_contacts_on = ['pelvis']
        collapse_fixed_joints = False # merge bodies connected by fixed joints. Specific fixed joints can be kept by adding " <... dont_collapse="true">
        flip_visual_attachments = False
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
        default_dof_drive_mode = 0 # see GymDofDriveModeFlags (0 is none, 1 is pos tgt, 2 is vel tgt, 3 effort)
        replace_cylinder_with_capsule = True # replace collision cylinders with capsules, leads to faster/more stable simulation

        density = 0.001
        angular_damping = 0.
        linear_damping = 0.
        max_angular_velocity = 100.
        max_linear_velocity = 100.
        armature = 0.
        thickness = 0.01

    class domain_rand( LeggedRobotCfg.domain_rand ):
        randomize_friction = False
        randomize_base_mass = False
        push_robots = False
  
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.95
        soft_dof_vel_limit = 0.9
        soft_torque_limit = 0.9
        max_contact_force = 300.
        only_positive_rewards = False

        class scales( LeggedRobotCfg.rewards.scales ):
            termination = -200.
            tracking_ang_vel = 1.0
            torques = -5.e-6
            dof_acc = -2.e-7
            lin_vel_z = -0.5
            feet_air_time = 5.
            dof_pos_limits = -1.
            no_fly = 0.25
            dof_vel = -0.0
            ang_vel_xy = -0.0
            feet_contact_forces = -0.

    class noise( LeggedRobotCfg.noise ):
        add_noise = False

class HopperCfgPPO( LeggedRobotCfgPPO ):

    class policy:
        init_noise_std = 1.0
        actor_hidden_dims = [512, 256, 128]
        critic_hidden_dims = [512, 256, 128]
        activation = 'elu' # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid
    
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'hopper'

    class algorithm( LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.01
        learning_rate = 1.e-3 #5.e-4



  