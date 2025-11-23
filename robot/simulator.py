"""MuJoCo robot simulator with automatic position control for Panda-Omron mobile manipulator."""

import time
import numpy as np
import mujoco, mujoco.viewer
from scipy.spatial.transform import Rotation


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

    # End effector site name
    EE_SITE_NAME = "gripper0_right_grip_site"

    # Mobile PID controller gains
    MOBILE_KP = np.array([4.0, 4.0, 2.0])
    MOBILE_KI = np.array([0.3, 0.3, 0.15])
    MOBILE_I_LIMIT = np.array([0.2, 0.2, 0.1])
    MOBILE_KD = np.array([0.5, 0.5, 0.3])

    # Arm PD controller gains for position control (7 joints)
    ARM_KP = np.array([100.0, 100.0, 100.0, 100.0, 50.0, 50.0, 50.0])
    ARM_KD = np.array([10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 5.0])
    ARM_JOINT_LIMITS = np.array([[-2.9, 2.9]] * 7)

    # IK solver parameters
    IK_MAX_ITERATIONS = 100
    IK_POSITION_TOLERANCE = 0.001  # 1mm
    IK_ORIENTATION_TOLERANCE = 0.01  # ~0.57 degrees
    IK_DAMPING = 0.01  # Damped Least Squares damping factor
    IK_STEP_SIZE = 0.5  # Step size for joint updates

    # Camera settings
    CAM_LOOKAT = [-0.8, -0.8, 0.8]
    CAM_DISTANCE = 7.5
    CAM_AZIMUTH = 135
    CAM_ELEVATION = -25

    MOBILE_INIT_POSITION = np.array([1.8, -3.45, 3.141592])
    ARM_INIT_POSITION = np.array([-0.0114, -1.0319,  0.0488, -2.2575,  0.0673,  1.5234, 0.6759])


class MujocoSimulator:
    """MuJoCo simulator with PD-controlled mobile base position tracking."""

    def __init__(self, xml_path="../model/robocasa/site.xml"):
        """Initialize simulator with MuJoCo model and control indices."""
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data  = mujoco.MjData(self.model)
        self._mobile_target_joint = RobotConfig.MOBILE_INIT_POSITION.copy()
        self._arm_target_joint = RobotConfig.ARM_INIT_POSITION.copy()
        self.dt = self.model.opt.timestep # PID timestep
        self._mobile_error_integral = np.zeros(3,) # I of PID

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
        self.arm_dof_indices = []
        for joint_id in self.arm_joint_ids:
            dof_adr = self.model.jnt_dofadr[joint_id]
            dof_num = self._get_joint_dof_count(joint_id)
            self.arm_dof_indices.extend(range(dof_adr, dof_adr + dof_num))

        # Resolve end effector site ID
        self.ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, RobotConfig.EE_SITE_NAME)
        # Body used to measure the mobile base pose in world coordinates
        self.mobile_base_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "mobilebase0_base"
        )
        
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

    def _get_joint_dof_count(self, joint_id):
        joint_type = self.model.jnt_type[joint_id]
        if joint_type == mujoco.mjtJoint.mjJNT_FREE:
            return 6
        if joint_type == mujoco.mjtJoint.mjJNT_BALL:
            return 3
        if joint_type in (mujoco.mjtJoint.mjJNT_SLIDE, mujoco.mjtJoint.mjJNT_HINGE):
            return 1
        raise ValueError(f"Unsupported joint type for joint_id {joint_id}")
    
    # ============================================================
    # Mobile Base Control Methods
    # ============================================================

    def get_mobile_world_position(self):
        """Get current mobile base pose [x, y, theta] in world frame."""
        base_pos = self.data.xpos[self.mobile_base_body_id]
        base_rot = self.data.xmat[self.mobile_base_body_id].reshape(3, 3)
        base_theta = np.arctan2(base_rot[1, 0], base_rot[0, 0])
        return np.array([base_pos[0], base_pos[1], base_theta])

    def get_mobile_target_joint(self):
        """Get current mobile base target joint [x, y, theta]."""
        return self._mobile_target_joint

    def set_mobile_target_joint(self, mobile_target_joint):
        """
        Set mobile base target joint [x, y, theta] in meters and radians.
        """
        self._mobile_target_joint = np.array(mobile_target_joint)
        self._mobile_error_integral[:] = 0

    def get_mobile_joint_position(self):
        """Get current mobile joint position [x, y, theta] from joint states."""
        return np.array([
            self.data.qpos[self.mobile_joint_ids[0]],
            self.data.qpos[self.mobile_joint_ids[1]],
            self.data.qpos[self.mobile_joint_ids[2]]
        ])

    def get_mobile_joint_diff(self):
        """Get mobile base position error [delta_x, delta_y, delta_theta] between target and current position."""
        return self._mobile_target_joint - self.get_mobile_joint_position()

    def get_mobile_joint_velocity(self):
        """Get current mobile base velocity [vx, vy, omega] from joint velocities."""
        return np.array([
            self.data.qvel[self.mobile_joint_ids[0]],
            self.data.qvel[self.mobile_joint_ids[1]],
            self.data.qvel[self.mobile_joint_ids[2]]
        ])

    def _compute_mobile_control(self):
        """Compute PD control commands [vx, vy, omega] for mobile base to reach target."""
        current_pos = self.get_mobile_joint_position()
        current_vel = self.get_mobile_joint_velocity()

        pos_error = self._mobile_target_joint - current_pos
        pos_error[2] = np.arctan2(np.sin(pos_error[2]), np.cos(pos_error[2]))  # Normalize angle
        self._mobile_error_integral += pos_error * self.dt
        self._mobile_error_integral = np.clip(
            self._mobile_error_integral,
            -RobotConfig.MOBILE_I_LIMIT,
            RobotConfig.MOBILE_I_LIMIT
        )

        p_term = RobotConfig.MOBILE_KP * pos_error
        i_term = RobotConfig.MOBILE_KI * self._mobile_error_integral
        d_term = RobotConfig.MOBILE_KD * current_vel

        pid_cmd = p_term + i_term - d_term
        return pid_cmd

    # ============================================================
    # Arm Joint Control Methods
    # ============================================================

    def get_arm_target_joint(self):
        """Get current arm target joint positions [j1~j7] in radians."""
        return self._arm_target_joint

    def set_arm_target_joint(self, arm_target_joint):
        """
        Set arm target joint positions [j1~j7] in radians.
        
        Args:
            arm_target_joint: Array-like of 7 joint angles in radians
        """
        self._arm_target_joint = np.array(arm_target_joint)

    def get_arm_joint_position(self):
        """Get current arm joint positions [j1~j7] from joint states."""
        return np.array([self.data.qpos[jid] for jid in self.arm_joint_ids])

    def get_arm_joint_diff(self):
        """Get arm position error [delta_j1~delta_j7] between target and current position."""
        return self._arm_target_joint - self.get_arm_joint_position()

    def get_arm_joint_velocity(self):
        """Get current arm joint velocities [v1~v7] from joint velocities."""
        return np.array([self.data.qvel[jid] for jid in self.arm_joint_ids])

    def _compute_arm_control(self):
        """
        Compute position control commands [j1~j7] for arm to reach target."""
        current_pos = self.get_arm_joint_position()
        current_vel = self.get_arm_joint_velocity()

        pos_error = self._arm_target_joint - current_pos

        return current_pos + RobotConfig.ARM_KP * pos_error - RobotConfig.ARM_KD * current_vel

    # ============================================================
    # End Effector Control Methods (Task Space)
    # ============================================================
    
    @staticmethod
    def _rotation_matrix_to_euler_xyz(R):
        """Convert rotation matrix to XYZ Euler angles [roll, pitch, yaw]."""
        return Rotation.from_matrix(R).as_euler("xyz")

    def get_ee_world_pose(self, data=None):
        """Return current end effector pose in world frame."""
        if data is None:
            data = self.data

        ee_pos = data.site_xpos[self.ee_site_id].copy()
        ee_rot = data.site_xmat[self.ee_site_id].reshape(3, 3)
        ee_ori = self._rotation_matrix_to_euler_xyz(ee_rot)
        return ee_pos, ee_ori

    def get_ee_position(self):
        """Return end effector pose relative to the mobile base frame."""
        ee_pos_world, ee_ori_world = self.get_ee_world_pose()
        base_pos_world = self.get_mobile_world_position()
        base_theta = base_pos_world[2]

        # Translate to base origin and rotate into base frame
        dx = ee_pos_world[0] - base_pos_world[0]
        dy = ee_pos_world[1] - base_pos_world[1]
        cos_theta = np.cos(-base_theta)
        sin_theta = np.sin(-base_theta)
        ee_pos_local = np.array([
            dx * cos_theta - dy * sin_theta,
            dx * sin_theta + dy * cos_theta,
            ee_pos_world[2]
        ])

        ee_ori_local = ee_ori_world.copy()
        ee_ori_local[2] -= base_theta
        ee_ori_local[2] = np.arctan2(np.sin(ee_ori_local[2]), np.cos(ee_ori_local[2]))
        return ee_pos_local, ee_ori_local

    def _compute_ee_jacobian(self, data=None):
        """Compute 6x7 Jacobian for the end effector site (arm joints only)."""
        if data is None:
            data = self.data

        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, data, jacp, jacr, self.ee_site_id)

        jacp_arm = jacp[:, self.arm_dof_indices]
        jacr_arm = jacr[:, self.arm_dof_indices]
        return np.vstack([jacp_arm, jacr_arm])

    def _solve_ik_position(self, target_pos, max_iterations=None):
        """Solve IK for a target position (orientation is kept constant)."""
        if max_iterations is None:
            max_iterations = RobotConfig.IK_MAX_ITERATIONS

        q = self.get_arm_joint_position().copy()

        ik_data = mujoco.MjData(self.model)
        ik_data.qpos[:] = self.data.qpos[:]

        for _ in range(max_iterations):
            for i, joint_id in enumerate(self.arm_joint_ids):
                ik_data.qpos[joint_id] = q[i]
            mujoco.mj_forward(self.model, ik_data)

            current_pos = ik_data.site_xpos[self.ee_site_id].copy()
            pos_error = target_pos - current_pos

            if np.linalg.norm(pos_error) < RobotConfig.IK_POSITION_TOLERANCE:
                return True, q

            jacobian = self._compute_ee_jacobian(ik_data)[:3, :]
            jjt = jacobian @ jacobian.T
            damping = (RobotConfig.IK_DAMPING ** 2) * np.eye(jacobian.shape[0])
            inv_term = np.linalg.inv(jjt + damping)
            dq = jacobian.T @ (inv_term @ pos_error)
            q += RobotConfig.IK_STEP_SIZE * dq
            q = np.clip(q, RobotConfig.ARM_JOINT_LIMITS[:, 0], RobotConfig.ARM_JOINT_LIMITS[:, 1])

        return False, q

    def move_ee_delta(self, delta_pos):
        """Move the end effector by (dx, dy, dz)."""
        delta_pos = np.asarray(delta_pos, dtype=float)
        if delta_pos.shape != (3,):
            raise ValueError("delta_pos must be length-3 iterable")

        ee_local, _ = self.get_ee_position()
        target_local = ee_local + delta_pos
        base_pose = self.get_mobile_world_position()
        cos_theta = np.cos(base_pose[2])
        sin_theta = np.sin(base_pose[2])
        target_world = np.array([
            base_pose[0] + target_local[0] * cos_theta - target_local[1] * sin_theta,
            base_pose[1] + target_local[0] * sin_theta + target_local[1] * cos_theta,
            target_local[2]
        ])

        success, joint_angles = self._solve_ik_position(target_world)
        if success:
            self.set_arm_target_joint(joint_angles)
        return success, joint_angles

    # ============================================================
    # Simulation Loop
    # ============================================================

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
