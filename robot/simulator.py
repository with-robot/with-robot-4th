"""MuJoCo robot simulator with automatic position control for Panda-Omron mobile manipulator."""

import time
import numpy as np
import mujoco, mujoco.viewer
from scipy.spatial.transform import Rotation as R
from typing import Optional, List, Tuple
from simulator_util import PathPlanner, GridMapUtils


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

    # Gripper actuator names (2-finger parallel gripper)
    GRIPPER_ACTUATOR_NAMES = [
        "gripper0_right_gripper_finger_joint1",
        "gripper0_right_gripper_finger_joint2"
    ]

    # Mobile PID controller gains
    MOBILE_KP = np.array([2.00, 2.00, 1.50])
    MOBILE_KI = np.array([0.30, 0.30, 0.01])
    MOBILE_I_LIMIT = np.array([0.60, 0.60, 0.02])
    MOBILE_KD = np.array([1.00, 1.00, 0.50])

    # Arm PID controller gains for position control (7 joints)
    ARM_KP = np.array([2.0, 2.0, 2.0, 2.0, 1.5, 1.0, 1.0])
    ARM_KI = np.array([0.1, 0.1, 0.1, 0.1, 0.05, 0.05, 0.05])
    ARM_I_LIMIT = np.array([0.2, 0.2, 0.2, 0.2, 0.1, 0.1, 0.1])
    ARM_KD = np.array([0.05, 0.05, 0.05, 0.05, 0.01, 0.01, 0.01])
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

    MOBILE_INIT_POSITION = np.array([-1.0, 1.0, 0.0])
    ARM_INIT_POSITION = np.array([-0.0114, -1.0319,  0.0488, -2.2575,  0.0673,  1.5234, 0.6759])
    GRIPPER_INIT_WIDTH = 0.08

    # Mobile base physical dimensions
    MOBILE_BASE_RADIUS = 0.35  # Approximate radius of mobile base footprint in meters

    # Grid map parameters
    GRID_SIZE = 0.1  # Grid cell size in meters


class MujocoSimulator:
    """MuJoCo simulator with PD-controlled mobile base position tracking."""

    def __init__(self, xml_path: str = "../model/robocasa/site.xml") -> None:
        """Initialize simulator with MuJoCo model and control indices."""
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data  = mujoco.MjData(self.model)
        self._mobile_target_position = RobotConfig.MOBILE_INIT_POSITION.copy()
        self._arm_target_joint = RobotConfig.ARM_INIT_POSITION.copy()
        self._gripper_target_width = RobotConfig.GRIPPER_INIT_WIDTH
        self.dt = self.model.opt.timestep # PID timestep
        self._mobile_error_integral = np.zeros(3,) # I of PID for mobile base
        self._arm_error_integral = np.zeros(7,) # I of PID for arm

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
        self.mobile_base_center_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "mobile_base_center"
        )

        # Resolve gripper actuator IDs
        self.gripper_actuator_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                                     for name in RobotConfig.GRIPPER_ACTUATOR_NAMES]

        # Resolve object IDs
        self.object_ids = []
        for i in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            if name and name.startswith("object_"):
                self.object_ids.append(i)
        
        # Set initial mobile base positions (qpos) and velocities (ctrl=0 for velocity control)
        for i, (joint_id, actuator_id) in enumerate(zip(self.mobile_joint_ids, self.mobile_actuator_ids)):
            self.data.qpos[joint_id] = RobotConfig.MOBILE_INIT_POSITION[i]
            self.data.ctrl[actuator_id] = 0.0  # velocity control is 0.0

        # Set initial joint positions (qpos) and actuator targets (ctrl)
        for i, (joint_id, actuator_id) in enumerate(zip(self.arm_joint_ids, self.arm_actuator_ids)):
            self.data.qpos[joint_id] = RobotConfig.ARM_INIT_POSITION[i]
            self.data.ctrl[actuator_id] = RobotConfig.ARM_INIT_POSITION[i]
        
        # Initialize grid map
        self.grid_map = np.load("grid_map.npy")

        # Cache floor geometry information to avoid repeated queries
        self._floor_geom_id = None
        self._floor_size = None
        self._floor_pos = None

        # Compute forward kinematics
        mujoco.mj_forward(self.model, self.data)

    def _get_joint_dof_count(self, joint_id: int) -> int:
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

    def get_mobile_position(self) -> np.ndarray:
        """Get current mobile base pose [x, y, theta] in world frame."""
        base_pos = self.data.site_xpos[self.mobile_base_center_id]
        base_rot = self.data.site_xmat[self.mobile_base_center_id].reshape(3, 3)
        base_theta = np.arctan2(base_rot[1, 0], base_rot[0, 0])
        return np.array([base_pos[0], base_pos[1], base_theta])

    def get_mobile_target_position(self) -> np.ndarray:
        """Get current mobile base target pose [x, y, theta] in world frame."""
        return self._mobile_target_position

    def set_mobile_target_position(self, mobile_target_position: np.ndarray) -> None:
        """Set mobile base target pose [x, y, theta] in world frame."""
        self._mobile_target_position = np.array(mobile_target_position)
        self._mobile_error_integral[:] = 0

    def get_mobile_position_diff(self) -> np.ndarray:
        """Get mobile base position error [delta_x, delta_y, delta_theta] between target and current position."""
        diff = self._mobile_target_position - self.get_mobile_position()
        diff[2] = np.arctan2(np.sin(diff[2]), np.cos(diff[2]))
        return diff

    def get_mobile_velocity(self) -> np.ndarray:
        """Get current mobile base velocity [vx, vy, omega] from joint velocities."""
        return np.array([
            self.data.qvel[self.mobile_joint_ids[0]],
            self.data.qvel[self.mobile_joint_ids[1]],
            self.data.qvel[self.mobile_joint_ids[2]]
        ])

    def _compute_mobile_control(self) -> np.ndarray:
        """Compute PD control commands [vx, vy, omega] for mobile base to reach target."""
        current_pos = self.get_mobile_position()
        current_vel = self.get_mobile_velocity()

        pos_error = self.get_mobile_position_diff()
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
    # Mobile Planning Methods
    # ============================================================

    def plan_mobile_path(self, target_pos: np.ndarray, simplify: bool = True) -> Optional[List[np.ndarray]]:
        """Plan path for mobile base to reach target position using A* algorithm."""
        # Ensure target_pos is array-like with 2 elements
        target_pos = np.array(target_pos[:2]) if len(target_pos) > 2 else np.array(target_pos)

        grid_size = RobotConfig.GRID_SIZE

        # Inflate obstacles by robot radius for collision-free planning
        inflated_map = PathPlanner.inflate_obstacles(
            self.grid_map,
            RobotConfig.MOBILE_BASE_RADIUS,
            grid_size
        )

        # Get current position
        current_joint = self.get_mobile_position()

        # Convert to grid coordinates
        start_grid = self._world_to_grid(current_joint[:2], grid_size)
        goal_grid = self._world_to_grid(target_pos[:2], grid_size)

        # Find the closest free cell to target in the INFLATED map
        # This ensures sufficient clearance at the final goal position
        if inflated_map[goal_grid[0], goal_grid[1]] == 1:
            # Target is in obstacle (safety inflated), find nearest free cell along axis
            adjusted_goal = PathPlanner.find_nearest_axial_free_cell(goal_grid, inflated_map)
            if adjusted_goal is None:
                return None
        else:
            # Target is already in safe free space
            adjusted_goal = goal_grid

        # Run A* search on inflated map to the adjusted goal
        # This ensures safe path planning while reaching as close as possible
        path_grid, closest_point = PathPlanner.astar_search(start_grid, adjusted_goal, inflated_map)
        
        if path_grid is None:
            return None

        # Simplify path if requested
        if simplify and len(path_grid) > 2:
            # First pass: Line-of-sight simplification
            path_grid = PathPlanner.simplify_path_line_of_sight(path_grid, inflated_map)
            
            # Second pass: Angle-based filtering
            path_grid = PathPlanner.simplify_path_angle_filter(path_grid)
            
            # Third pass: B-spline smoothing
            path_grid = PathPlanner.smooth_path_bspline(path_grid)
        
        # Convert grid path to world coordinates
        path_world = []
        for i, grid_pos in enumerate(path_grid):
            world_xy = self._grid_to_world(grid_pos, grid_size)
            
            # Calculate orientation (theta)
            if i < len(path_grid) - 1:
                # Point towards next waypoint
                next_xy = self._grid_to_world(path_grid[i + 1], grid_size)
                theta = np.arctan2(next_xy[1] - world_xy[1], next_xy[0] - world_xy[0])
            elif i > 0:
                # Last waypoint: use direction from previous waypoint (natural arrival)
                prev_xy = self._grid_to_world(path_grid[i - 1], grid_size)
                theta = np.arctan2(world_xy[1] - prev_xy[1], world_xy[0] - prev_xy[0])
            else:
                # Single waypoint: point towards original target
                theta = np.arctan2(target_pos[1] - world_xy[1], target_pos[0] - world_xy[0])
            
            path_world.append(np.array([world_xy[0], world_xy[1], theta]))

        # Add final rotation waypoint to face the original target
        if len(path_world) > 0:
            last_pos = path_world[-1][:2]  # [x, y] of last waypoint
            target_theta = np.arctan2(target_pos[1] - last_pos[1], target_pos[0] - last_pos[0])

            # Add rotation waypoint (same position, different orientation)
            path_world.append(np.array([last_pos[0], last_pos[1], target_theta]))

        return path_world
    
    def follow_mobile_path(self, path_world: List[np.ndarray], timeout_per_waypoint: float = 30.0, verbose: bool = False) -> bool:
        """Follow a path by sequentially moving to each waypoint."""
        if verbose:
            print(f"Following path with {len(path_world)} waypoints")
        
        for i, waypoint in enumerate(path_world):
            if verbose:
                print(f"Moving to waypoint {i+1}/{len(path_world)}: [{waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f}]")
            
            # Check if this is the last waypoint
            is_last_waypoint = (i == len(path_world) - 1)

            # Get current position
            curr_pos = self.get_mobile_position()

            # Calculate angle difference
            angle_diff = waypoint[2] - curr_pos[2]
            angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi

            if abs(angle_diff) > np.deg2rad(45):
                rotate_target = curr_pos.copy()
                rotate_target[2] = waypoint[2]
                self.set_mobile_target_position(rotate_target)

                # Wait for rotation to complete
                start_time = time.time()
                while time.time() - start_time < 5.0:
                    if np.abs(self.get_mobile_position_diff()[2]) < np.deg2rad(5):
                        break
                    time.sleep(0.02)

            self.set_mobile_target_position(waypoint)
                      
            # Wait for convergence
            start_time = time.time()
            converged = False
            
            while time.time() - start_time < timeout_per_waypoint:
                # Check position and velocity convergence
                pos_diff = self.get_mobile_position_diff()
                pos_diff[-1] /= 2  # Theta weighted at 50%
                pos_error = np.linalg.norm(pos_diff)
                vel_error = np.linalg.norm(self.get_mobile_velocity())
                
                if is_last_waypoint:
                    # Last waypoint: Strict stop required
                    if pos_error < 0.05 and vel_error < 0.05:
                        converged = True
                        if verbose:
                            print(f"  Reached destination in {time.time() - start_time:.2f}s")
                        break
                else:
                    # Intermediate waypoints: Pass through without stopping (no velocity check)
                    if pos_error < 0.15:
                        converged = True
                        break
                
                time.sleep(0.02)
            
            if not converged:
                if verbose:
                    print(f"  Timeout at waypoint {i+1} (pos_error={pos_error:.4f}, vel_error={vel_error:.4f})")
                return False
        
        if verbose:
            print("Path following completed successfully")
        return True

    # ============================================================
    # Arm Joint Control Methods
    # ============================================================

    def get_arm_target_joint(self) -> np.ndarray:
        """Get current arm target joint positions [j1~j7] in radians."""
        return self._arm_target_joint

    def set_arm_target_joint(self, arm_target_joint: np.ndarray) -> None:
        """Set arm target joint positions [j1~j7] in radians."""
        self._arm_target_joint = np.array(arm_target_joint)
        self._arm_error_integral[:] = 0

    def get_arm_joint_position(self) -> np.ndarray:
        """Get current arm joint positions [j1~j7] from joint states."""
        return np.array([self.data.qpos[jid] for jid in self.arm_joint_ids])

    def get_arm_joint_diff(self) -> np.ndarray:
        """Get arm position error [delta_j1~delta_j7] between target and current position."""
        return self._arm_target_joint - self.get_arm_joint_position()

    def get_arm_joint_velocity(self) -> np.ndarray:
        """Get current arm joint velocities [v1~v7] from joint velocities."""
        return np.array([self.data.qvel[jid] for jid in self.arm_joint_ids])

    def _compute_arm_control(self) -> np.ndarray:
        """Compute PID position control commands [j1~j7] for arm to reach target."""
        current_pos = self.get_arm_joint_position()
        current_vel = self.get_arm_joint_velocity()

        pos_error = self._arm_target_joint - current_pos
        
        # Update integral term with anti-windup
        self._arm_error_integral += pos_error * self.dt
        self._arm_error_integral = np.clip(
            self._arm_error_integral,
            -RobotConfig.ARM_I_LIMIT,
            RobotConfig.ARM_I_LIMIT
        )

        p_term = RobotConfig.ARM_KP * pos_error
        i_term = RobotConfig.ARM_KI * self._arm_error_integral
        d_term = RobotConfig.ARM_KD * current_vel

        return current_pos + p_term + i_term - d_term

    # ============================================================
    # End Effector Control Methods
    # ============================================================
    
    @staticmethod
    def _rotation_matrix_to_euler_xyz(rot: np.ndarray) -> np.ndarray:
        """Convert rotation matrix to XYZ Euler angles [roll, pitch, yaw]."""
        return R.from_matrix(rot.reshape(3, 3)).as_euler("xyz")

    def get_ee_position(self, data: Optional[mujoco.MjData] = None) -> Tuple[np.ndarray, np.ndarray]:
        """Return current end effector position and orientation in world frame."""
        if data is None:
            data = self.data

        ee_pos = data.site_xpos[self.ee_site_id].copy()
        ee_rot = data.site_xmat[self.ee_site_id]
        ee_ori = self._rotation_matrix_to_euler_xyz(ee_rot)
        return ee_pos, ee_ori

    def _compute_ee_jacobian(self, data: Optional[mujoco.MjData] = None) -> np.ndarray:
        """Compute 6x7 Jacobian for the end effector site (arm joints only)."""
        if data is None:
            data = self.data

        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, data, jacp, jacr, self.ee_site_id)

        jacp_arm = jacp[:, self.arm_dof_indices]
        jacr_arm = jacr[:, self.arm_dof_indices]
        return np.vstack([jacp_arm, jacr_arm])

    def _solve_ik_position(self, target_pos: np.ndarray, max_iterations: Optional[int] = None) -> Tuple[bool, np.ndarray]:
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
    
    def set_ee_target_position(self, target_pos: np.ndarray) -> Tuple[bool, np.ndarray]:
        """Set end effector target position in world frame."""
        success, joint_angles = self._solve_ik_position(target_pos)
        if success:
            self.set_arm_target_joint(joint_angles)
        return success, joint_angles

    # ============================================================
    # Gripper Control Methods
    # ============================================================

    def get_gripper_width(self) -> float:
        """Get current gripper width in meters."""
        return 2.0 * self.data.ctrl[self.gripper_actuator_ids[0]]
    
    def set_target_gripper_width(self, width: float) -> None:
        """Set target gripper width in meters (0.0 = closed, 0.08 = fully open)."""
        self._gripper_target_width = np.clip(width, 0.0, 0.08)
    
    def get_gripper_width_diff(self) -> float:
        """Get gripper width error between target and current position."""
        return self._gripper_target_width - self.get_gripper_width()
    
    def get_gripper_width_velocity(self) -> float:
        """Get gripper width velocity."""
        return self.data.ctrl[self.gripper_actuator_ids[0]]
    
    def _compute_gripper_control(self) -> np.ndarray:
        """Compute gripper control commands."""
        # Target width is symmetric: finger1 = +width/2, finger2 = -width/2
        target_finger1 = self._gripper_target_width / 2.0
        target_finger2 = -self._gripper_target_width / 2.0
        
        return np.array([target_finger1, target_finger2])
    
    # ============================================================
    # Pick & Place Methods
    # ============================================================

    def _wait_for_arm_convergence(self, timeout: float = 10.0) -> bool:
        """Wait for arm to converge to target position."""
        start_time = time.time()
        while time.time() - start_time < timeout:
            pos_error = np.linalg.norm(self.get_arm_joint_diff())
            vel_error = np.linalg.norm(self.get_arm_joint_velocity())
            if pos_error < 0.1 and vel_error < 0.1:
                return True
            time.sleep(0.02)
        return False

    def pick_object(
        self, 
        object_pos: np.ndarray, 
        approach_height: float = 0.1, 
        lift_height: float = 0.2,
        return_to_home: bool = True,
        timeout: float = 10.0,
        verbose: bool = False
    ) -> bool:
        """Pick up an object at the specified position."""
        if verbose:
            print(f"Starting pick sequence at position [{object_pos[0]:.3f}, {object_pos[1]:.3f}, {object_pos[2]:.3f}]")
        
        # Step 1: Open gripper
        if verbose:
            print("  Step 1: Opening gripper...")
        self.set_target_gripper_width(0.08)
        time.sleep(1.0)
        
        # Step 2: Move to approach position (above object)
        approach_pos = np.array([object_pos[0], object_pos[1], object_pos[2] + approach_height])
        if verbose:
            print(f"  Step 2: Moving to approach position (height: {approach_height:.3f}m above object)...")
        success, _ = self.set_ee_target_position(approach_pos)
        if not success:
            if verbose:
                print("  Failed to reach approach position")
            return False
        
        if not self._wait_for_arm_convergence(timeout):
            if verbose:
                print("  Timeout waiting for approach position")
            return False
        
        # Step 3: Lower to grasp position
        grasp_pos = np.array([object_pos[0], object_pos[1], object_pos[2]])
        if verbose:
            print(f"  Step 3: Lowering to grasp position...")
        success, _ = self.set_ee_target_position(grasp_pos)
        if not success:
            if verbose:
                print("  Failed to reach grasp position")
            return False
        
        if not self._wait_for_arm_convergence(timeout):
            if verbose:
                print("  Timeout waiting for grasp position")
            return False
        
        # Step 4: Close gripper to grasp
        if verbose:
            print("  Step 4: Closing gripper to grasp...")
        self.set_target_gripper_width(0.02)
        time.sleep(1.5)  # Wait for gripper to close and stabilize
        
        # Step 5: Lift object
        lift_pos = np.array([object_pos[0], object_pos[1], object_pos[2] + lift_height])
        if verbose:
            print(f"  Step 5: Lifting object (height: {lift_height:.3f}m above original position)...")
        success, _ = self.set_ee_target_position(lift_pos)
        if not success:
            if verbose:
                print("  Failed to lift object")
            return False
        
        if not self._wait_for_arm_convergence(timeout):
            if verbose:
                print("  Timeout waiting for lift position")
            return False
        
        # Step 6: Return to home position (optional)
        if return_to_home:
            if verbose:
                print("  Step 6: Returning arm to home position...")
            self.set_arm_target_joint(RobotConfig.ARM_INIT_POSITION)
            
            if not self._wait_for_arm_convergence(timeout):
                if verbose:
                    print("  Timeout waiting for home position")
                return False
        
        if verbose:
            print("  Pick sequence completed successfully!")
        return True
    
    def place_object(
        self,
        place_pos: np.ndarray,
        approach_height: float = 0.2,
        retract_height: float = 0.3,
        return_to_home: bool = True,
        timeout: float = 10.0,
        verbose: bool = False
    ) -> bool:
        """Place an object at the specified position."""
        if verbose:
            print(f"Starting place sequence at position [{place_pos[0]:.3f}, {place_pos[1]:.3f}, {place_pos[2]:.3f}]")
        
        # Step 1: Move to approach position (above placement location)
        approach_pos = np.array([place_pos[0], place_pos[1], place_pos[2] + approach_height])
        if verbose:
            print(f"  Step 1: Moving to approach position (height: {approach_height:.3f}m above target)...")
        success, _ = self.set_ee_target_position(approach_pos)
        if not success:
            if verbose:
                print("  Failed to reach approach position")
            return False
        
        if not self._wait_for_arm_convergence(timeout):
            if verbose:
                print("  Timeout waiting for approach position")
            return False
        
        # Step 2: Open gripper to release
        if verbose:
            print("  Step 2: Opening gripper to release object...")
        self.set_target_gripper_width(0.08)
        time.sleep(1.5)  # Wait for gripper to open and object to settle
        
        # Step 3: Retract upward
        retract_pos = np.array([place_pos[0], place_pos[1], place_pos[2] + retract_height])
        if verbose:
            print(f"  Step 3: Retracting (height: {retract_height:.3f}m above placement)...")
        success, _ = self.set_ee_target_position(retract_pos)
        if not success:
            if verbose:
                print("  Failed to retract")
            return False
        
        if not self._wait_for_arm_convergence(timeout):
            if verbose:
                print("  Timeout waiting for retract position")
            return False
        
        # Step 4: Return to home position (optional)
        if return_to_home:
            if verbose:
                print("  Step 4: Returning arm to home position...")
            self.set_arm_target_joint(RobotConfig.ARM_INIT_POSITION)
            
            if not self._wait_for_arm_convergence(timeout):
                if verbose:
                    print("  Timeout waiting for home position")
                return False
        
        if verbose:
            print("  Place sequence completed successfully!")
        return True
    
    # ============================================================
    # Object Interaction Methods
    # ============================================================

    def get_object_positions(self) -> dict:
        """Get list of object dictionaries with id, name, position and orientation in world frame."""
        objects = {}
        for i in self.object_ids:
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            if name and name.startswith("object_"):
                objects[name] = {
                    'id': i,
                    'pos': self.data.xpos[i], 
                    'ori': self._rotation_matrix_to_euler_xyz(self.data.xmat[i])
                }
        return objects

    # ============================================================
    # Grid Map Methods
    # ============================================================

    def get_grid_map(self) -> np.ndarray:
        """Get grid map of the environment.

        Returns:
            np.ndarray: Binary occupancy grid (0=free, 1=occupied)
        """
        return self.grid_map

    def _get_floor_info(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get cached floor geometry information.

        Returns:
            Tuple[np.ndarray, np.ndarray]: (floor_size, floor_pos) where floor_pos may change
        """
        if self._floor_geom_id is None:
            self._floor_geom_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor_room_g0"
            )
            self._floor_size = self.model.geom_size[self._floor_geom_id]
        # Floor position may change, so always get from data
        self._floor_pos = self.data.geom_xpos[self._floor_geom_id]
        return self._floor_size, self._floor_pos

    def _world_to_grid(self, world_pos: Tuple[float, float], grid_size: float = RobotConfig.GRID_SIZE) -> Tuple[int, int]:
        """Convert world position [x, y] to grid indices [i, j].

        Args:
            world_pos: World position (x, y) in meters
            grid_size: Grid cell size in meters (default: RobotConfig.GRID_SIZE)

        Returns:
            Tuple[int, int]: Grid indices (i, j)
        """
        _, floor_pos = self._get_floor_info()
        return GridMapUtils.world_to_grid(world_pos, floor_pos, self.grid_map.shape, grid_size)

    def _grid_to_world(self, grid_pos: Tuple[int, int], grid_size: float = RobotConfig.GRID_SIZE) -> np.ndarray:
        """Convert grid indices [i, j] to world position [x, y].

        Args:
            grid_pos: Grid indices (i, j)
            grid_size: Grid cell size in meters (default: RobotConfig.GRID_SIZE)

        Returns:
            np.ndarray: World position [x, y] in meters
        """
        _, floor_pos = self._get_floor_info()
        return GridMapUtils.grid_to_world(grid_pos, floor_pos, self.grid_map.shape, grid_size)

    # ============================================================
    # Simulation Loop
    # ============================================================

    def run(self) -> None:
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

                # gripper control
                gripper_control = self._compute_gripper_control()
                for i, actuator_id in enumerate(self.gripper_actuator_ids):
                    self.data.ctrl[actuator_id] = gripper_control[i]

                mujoco.mj_step(self.model, self.data)
                v.sync()
