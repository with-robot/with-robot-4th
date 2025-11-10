"""MuJoCo robot simulator with automatic position control for Panda-Omron mobile manipulator."""

import time
import numpy as np
import mujoco, mujoco.viewer


class RobotConfig:
    """Robot simulation configuration constants."""

    # Mobile base joints: [x, y, theta]
    MOBILE_JOINT_NAMES = [
        "mobilebase0_joint_mobile_side",
        "mobilebase0_joint_mobile_forward",
        "mobilebase0_joint_mobile_yaw"
    ]

    MOBILE_ACTUATOR_NAMES = [
        "mobilebase0_actuator_mobile_side",
        "mobilebase0_actuator_mobile_forward",
        "mobilebase0_actuator_mobile_yaw"
    ]

    # Panda arm joints: [joint1 ~ joint7]
    ARM_JOINT_NAMES = [
        "robot0_joint1",
        "robot0_joint2",
        "robot0_joint3",
        "robot0_joint4",
        "robot0_joint5",
        "robot0_joint6",
        "robot0_joint7"
    ]

    ARM_ACTUATOR_NAMES = [
        "robot0_torq_j1",
        "robot0_torq_j2",
        "robot0_torq_j3",
        "robot0_torq_j4",
        "robot0_torq_j5",
        "robot0_torq_j6",
        "robot0_torq_j7"
    ]

    # Mobile PD controller gains: [kp_x, kp_y, kp_theta]
    MOBILE_KP = np.array([2.0, 2.0, 1.5])
    MOBILE_KD = np.array([0.5, 0.5, 0.3])

    # Arm PD controller gains for position control (7 joints)
    ARM_KP = np.array([100.0, 100.0, 100.0, 100.0, 50.0, 50.0, 50.0])
    ARM_KD = np.array([10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 5.0])

    # Camera settings
    CAM_LOOKAT = [-0.8, -0.8, 0.8]
    CAM_DISTANCE = 7.5
    CAM_AZIMUTH = 135
    CAM_ELEVATION = -25

    MOBILE_INIT_POSITION = np.array([0.0, 0.0, 0.0])
    ARM_INIT_POSITION = np.array([0.0, 0.0, 0.0, 0.0, 0.0, np.pi/2, np.pi/4])


class MujocoSimulator:
    """MuJoCo simulator with PD-controlled mobile base position tracking."""

    def __init__(self, xml_path="../model/robocasa/panda_omron.xml"):
        """Initialize simulator with MuJoCo model and control indices."""
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data  = mujoco.MjData(self.model)
        self._mobile_target_position = RobotConfig.MOBILE_INIT_POSITION.copy()
        self._arm_target_position = RobotConfig.ARM_INIT_POSITION.copy()

        # Resolve joint/actuator names to indices
        self.mobile_joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                                 for name in RobotConfig.MOBILE_JOINT_NAMES]
        self.mobile_actuator_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                                    for name in RobotConfig.MOBILE_ACTUATOR_NAMES]
        
        # Resolve Panda arm joint IDs and set initial positions
        self.arm_joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                              for name in RobotConfig.ARM_JOINT_NAMES]
        self.arm_actuator_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                                 for name in RobotConfig.ARM_ACTUATOR_NAMES]
        
        # Set initial mobile base positions (qpos) and velocities (ctrl=0 for velocity control)
        for i, (joint_id, actuator_id) in enumerate(zip(self.mobile_joint_ids, self.mobile_actuator_ids)):
            self.data.qpos[joint_id] = RobotConfig.MOBILE_INIT_POSITION[i]
            self.data.ctrl[actuator_id] = 0.0  # velocity control is 0.0

        # Set initial joint positions (qpos) and actuator targets (ctrl)
        for i, (joint_id, actuator_id) in enumerate(zip(self.arm_joint_ids, self.arm_actuator_ids)):
            self.data.qpos[joint_id] = RobotConfig.ARM_INIT_POSITION[i]
            self.data.ctrl[actuator_id] = RobotConfig.ARM_INIT_POSITION[i]
        
        # Forward kinematics 계산
        mujoco.mj_forward(self.model, self.data)

    def get_mobile_target_position(self):
        """Get current mobile base target position [x, y, theta]."""
        return self._mobile_target_position

    def set_mobile_target_position(self, mobile_target_position):
        """
        Set mobile base target position [x, y, theta] in meters and radians.
        """
        self._mobile_target_position = np.array(mobile_target_position)

    def get_current_mobile_position(self):
        """Get current mobile base position [x, y, theta] from joint states."""
        return np.array([
            self.data.qpos[self.mobile_joint_ids[0]],
            self.data.qpos[self.mobile_joint_ids[1]],
            self.data.qpos[self.mobile_joint_ids[2]]
        ])

    def get_mobile_position_diff(self):
        """Get mobile base position error [delta_x, delta_y, delta_theta] between target and current position."""
        return self._mobile_target_position - self.get_current_mobile_position()

    def get_current_mobile_velocity(self):
        """Get current mobile base velocity [vx, vy, omega] from joint velocities."""
        return np.array([
            self.data.qvel[self.mobile_joint_ids[0]],
            self.data.qvel[self.mobile_joint_ids[1]],
            self.data.qvel[self.mobile_joint_ids[2]]
        ])

    def _compute_mobile_control(self):
        """Compute PD control commands [vx, vy, omega] for mobile base to reach target."""
        current_pos = self.get_current_mobile_position()
        current_vel = self.get_current_mobile_velocity()

        pos_error = self._mobile_target_position - current_pos
        pos_error[2] = np.arctan2(np.sin(pos_error[2]), np.cos(pos_error[2]))  # Normalize angle

        return RobotConfig.MOBILE_KP * pos_error - RobotConfig.MOBILE_KD * current_vel

    def get_arm_target_position(self):
        """Get current arm target joint positions [j1~j7] in radians."""
        return self._arm_target_position

    def set_arm_target_position(self, joint_positions):
        """
        Set arm target joint positions [j1~j7] in radians.
        
        Args:
            joint_positions: Array-like of 7 joint angles in radians
        """
        self._arm_target_position = np.array(joint_positions)

    def get_current_arm_position(self):
        """Get current arm joint positions [j1~j7] from joint states."""
        return np.array([self.data.qpos[jid] for jid in self.arm_joint_ids])

    def get_arm_position_diff(self):
        """Get arm position error [delta_j1~delta_j7] between target and current position."""
        return self._arm_target_position - self.get_current_arm_position()

    def get_current_arm_velocity(self):
        """Get current arm joint velocities [v1~v7] from joint velocities."""
        return np.array([self.data.qvel[jid] for jid in self.arm_joint_ids])

    def _compute_arm_control(self):
        """
        Compute position control commands [j1~j7] for arm to reach target."""
        current_pos = self.get_current_arm_position()
        current_vel = self.get_current_arm_velocity()

        pos_error = self._arm_target_position - current_pos

        return current_pos + RobotConfig.ARM_KP * pos_error - RobotConfig.ARM_KD * current_vel

    def run(self):
        """Run simulation with 3D viewer and PD control loop (blocking)."""
        with mujoco.viewer.launch_passive(self.model, self.data) as v:
            # Camera setup
            v.cam.lookat[:] = RobotConfig.CAM_LOOKAT
            v.cam.distance = RobotConfig.CAM_DISTANCE
            v.cam.azimuth = RobotConfig.CAM_AZIMUTH
            v.cam.elevation = RobotConfig.CAM_ELEVATION

            # Hide debug visuals
            v.opt.geomgroup[0] = 0
            v.opt.sitegroup[0] = v.opt.sitegroup[1] = v.opt.sitegroup[2] = 0
            v.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False
            v.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
            v.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = False
            v.opt.flags[mujoco.mjtVisFlag.mjVIS_PERTFORCE] = False
            v.opt.flags[mujoco.mjtVisFlag.mjVIS_PERTOBJ] = False
            v.opt.frame = mujoco.mjtFrame.mjFRAME_NONE
            v.opt.label = mujoco.mjtLabel.mjLABEL_NONE

            # Main loop
            while v.is_running():
                # mobile base control
                mobile_control = self._compute_mobile_control()
                self.data.ctrl[self.mobile_actuator_ids[0]] = mobile_control[0]
                self.data.ctrl[self.mobile_actuator_ids[1]] = mobile_control[1]
                self.data.ctrl[self.mobile_actuator_ids[2]] = mobile_control[2]

                # arm control
                arm_control = self._compute_arm_control()
                for i, actuator_id in enumerate(self.arm_actuator_ids):
                    self.data.ctrl[actuator_id] = arm_control[i]

                mujoco.mj_step(self.model, self.data)
                v.sync()
