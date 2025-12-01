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

    MOBILE_INIT_POSITION = np.array([-1.0, 1.0, np.pi])
    ARM_INIT_POSITION = np.array([-0.0114, -1.0319,  0.0488, -2.2575,  0.0673,  1.5234, 0.6759])
    GRIPPER_INIT_WIDTH = 0.08

    # Mobile base physical dimensions
    MOBILE_BASE_RADIUS = 0.3  # Approximate radius of mobile base footprint in meters

    # Grid map parameters
    GRID_SIZE = 0.1  # Grid cell size in meters


class MujocoSimulator:
    """MuJoCo simulator with PD-controlled mobile base position tracking."""

    def __init__(self, xml_path: str = "../model/robocasa/site.xml") -> None:
        """Initialize simulator with MuJoCo model and control indices."""
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data  = mujoco.MjData(self.model)
        self._mobile_target_joint = RobotConfig.MOBILE_INIT_POSITION.copy()
        self._arm_target_joint = RobotConfig.ARM_INIT_POSITION.copy()
        self._gripper_target_width = RobotConfig.GRIPPER_INIT_WIDTH
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
        self.grid_map = None

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

    def get_mobile_world_position(self) -> np.ndarray:
        """Get current mobile base pose [x, y, theta] in world frame."""
        base_pos = self.data.xpos[self.mobile_base_body_id]
        base_rot = self.data.xmat[self.mobile_base_body_id].reshape(3, 3)
        base_theta = np.arctan2(base_rot[1, 0], base_rot[0, 0])
        return np.array([base_pos[0], base_pos[1], base_theta])

    def get_mobile_target_joint(self) -> np.ndarray:
        """Get current mobile base target joint [x, y, theta]."""
        return self._mobile_target_joint

    def set_mobile_target_joint(self, mobile_target_joint: np.ndarray) -> None:
        """Set mobile base target joint [x, y, theta] in meters and radians."""
        self._mobile_target_joint = np.array(mobile_target_joint)
        self._mobile_error_integral[:] = 0

    def get_mobile_joint_position(self) -> np.ndarray:
        """Get current mobile joint position [x, y, theta] from joint states."""
        return np.array([
            self.data.qpos[self.mobile_joint_ids[0]],
            self.data.qpos[self.mobile_joint_ids[1]],
            self.data.qpos[self.mobile_joint_ids[2]]
        ])

    def get_mobile_joint_diff(self) -> np.ndarray:
        """Get mobile base position error [delta_x, delta_y, delta_theta] between target and current position."""
        return self._mobile_target_joint - self.get_mobile_joint_position()

    def get_mobile_joint_velocity(self) -> np.ndarray:
        """Get current mobile base velocity [vx, vy, omega] from joint velocities."""
        return np.array([
            self.data.qvel[self.mobile_joint_ids[0]],
            self.data.qvel[self.mobile_joint_ids[1]],
            self.data.qvel[self.mobile_joint_ids[2]]
        ])

    def _compute_mobile_control(self) -> np.ndarray:
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
    # Mobile Planning Methods
    # ============================================================

    def plan_mobile_path(self, target_pos: np.ndarray, grid_size: float = RobotConfig.GRID_SIZE, simplify: bool = True) -> Optional[List[np.ndarray]]:
        """Plan path for mobile base to reach target position using A* algorithm.

        Args:
            target_pos: Target [x, y] position (theta is automatically calculated)
            grid_size: Grid cell size in meters (default: RobotConfig.GRID_SIZE)
            simplify: Whether to simplify the path (default: True)

        Returns:
            Optional[List[np.ndarray]]: List of waypoints [x, y, theta] or None if no path found
        """
        # Ensure target_pos is array-like with 2 elements
        target_pos = np.array(target_pos[:2]) if len(target_pos) > 2 else np.array(target_pos)

        # Ensure grid map is initialized
        if self.grid_map is None:
            self.grid_map = self._make_grid_map(grid_size)
        
        # Inflate obstacles by robot radius for collision-free planning
        inflated_map = PathPlanner.inflate_obstacles(
            self.grid_map,
            RobotConfig.MOBILE_BASE_RADIUS,
            grid_size
        )

        # Get current position
        current_joint = self.get_mobile_joint_position()

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

    # ============================================================
    # Arm Joint Control Methods
    # ============================================================

    def get_arm_target_joint(self) -> np.ndarray:
        """Get current arm target joint positions [j1~j7] in radians."""
        return self._arm_target_joint

    def set_arm_target_joint(self, arm_target_joint: np.ndarray) -> None:
        """Set arm target joint positions [j1~j7] in radians."""
        self._arm_target_joint = np.array(arm_target_joint)

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
        """Compute position control commands [j1~j7] for arm to reach target."""
        current_pos = self.get_arm_joint_position()
        current_vel = self.get_arm_joint_velocity()

        pos_error = self._arm_target_joint - current_pos

        p_term = RobotConfig.ARM_KP * pos_error
        d_term = RobotConfig.ARM_KD * current_vel

        return current_pos + p_term - d_term

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

    def _make_grid_map(self, grid_size: float = RobotConfig.GRID_SIZE) -> np.ndarray:
        """Make grid map of the environment.

        Args:
            grid_size: Grid cell size in meters (default: RobotConfig.GRID_SIZE)

        Returns:
            np.ndarray: Binary occupancy grid (0=free, 1=occupied)
        """
        # Grid: main floor
        floor_size, floor_pos = self._get_floor_info()
        width = int(np.ceil((floor_size[0] * 2) / grid_size))   # Number of cells in x direction
        height = int(np.ceil((floor_size[1] * 2) / grid_size))  # Number of cells in y direction
        grid = np.zeros((height, width), dtype=np.uint8)

        def fill_geom_occupancy_by_id(geom_id):
            """Fill grid cells occupied by geometry"""
            # Get geometry properties
            geom_size = self.model.geom_size[geom_id]
            geom_pos = self.data.geom_xpos[geom_id]
            geom_mat = self.data.geom_xmat[geom_id].reshape(3, 3)

            # Define local corners of box geometry
            hx, hy, hz = geom_size
            local_corners = np.array([
                [ hx,  hy,  hz], [ hx,  hy, -hz],
                [ hx, -hy,  hz], [ hx, -hy, -hz],
                [-hx,  hy,  hz], [-hx,  hy, -hz],
                [-hx, -hy,  hz], [-hx, -hy, -hz],
            ])

            # Transform to world coordinates
            corners = local_corners @ geom_mat.T + geom_pos

            # Calculate floor bounds in world coordinates
            floor_half_x, floor_half_y, _ = floor_size
            floor_min_x = floor_pos[0] - floor_half_x
            floor_min_y = floor_pos[1] - floor_half_y

            # Project geometry corners to XY plane
            geom_xy = corners[:, :2]
            geom_min_x, geom_min_y = geom_xy.min(axis=0)
            geom_max_x, geom_max_y = geom_xy.max(axis=0)

            # Convert world coordinates to grid indices
            # Note: y-axis is inverted (higher y in world = lower row index in grid)
            floor_max_y = floor_pos[1] + floor_half_y
            j0 = int(np.floor((geom_min_x - floor_min_x) / grid_size))
            j1 = int(np.ceil((geom_max_x - floor_min_x) / grid_size))
            i0 = int(np.floor((floor_max_y - geom_max_y) / grid_size))
            i1 = int(np.ceil((floor_max_y - geom_min_y) / grid_size))

            # Clamp to grid bounds
            j0 = max(0, min(width, j0))
            j1 = max(0, min(width, j1))
            i0 = max(0, min(height, i0))
            i1 = max(0, min(height, i1))

            # Ensure at least one cell is occupied
            if i0 == i1:
                if i1 < height:
                    i1 += 1
                elif i0 > 0:
                    i0 -= 1
            if j0 == j1:
                if j1 < width:
                    j1 += 1
                elif j0 > 0:
                    j0 -= 1

            # Mark occupied cells
            grid[i0:i1, j0:j1] = 1
        
        def fill_body_occupancy(body_name):
            """Fill grid cells occupied by body"""
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            parentid = self.model.body_parentid
            nbody = self.model.nbody

            subtree = set()
            stack = [body_id]
            while stack:
                b = stack.pop()
                if b in subtree:
                    continue
                subtree.add(b)
                # Push child bodies of b
                for child in range(nbody):
                    if parentid[child] == b:
                        stack.append(child)
            
            for geom_id in range(self.model.ngeom):
                if self.model.geom_bodyid[geom_id] in subtree:
                    fill_geom_occupancy_by_id(geom_id)
            
        body_names = [
            "wall_room_main",
            "wall_backing_room_main",
            "wall_left_room_main",
            "wall_left_backing_room_main",
            "outlet_room_main",
            "outlet_2_room_main",
            "light_switch_room_main",
            "light_switch_2_room_main",
            "counter_corner_main_main_group_main",
            "counter_main_main_group_main",
            "stovetop_main_group_main",
            "counter_1_right_main_group_main",
            "fridge_main_group_main",
            "fridge_housing_main_group_main",
            "stack_1_main_group_base_main",
            "stack_1_main_group_1_main",
            "stack_1_main_group_2_main",
            "stack_1_main_group_3_main",
            "stack_1_main_group_4_main",
            "stack_2_main_group_base_main",
            "stack_2_main_group_1_main",
            "stack_2_main_group_2_main",
            "stack_3_main_group_base_main",
            "stack_3_main_group_1_main",
            "stack_3_main_group_2_main",
            "stack_3_main_group_3_main",
            "hood_main_group_main",
            "cab_main_main_group_main",
            "shelves_main_group_main",
            "fridge_cab_main_group_main",
            "toaster_main_group_main",
            "utensil_holder_main_group_main",
            "cab_1_left_group_main",
            "window_group_left_group_root",
            "cab_2_left_group_main",
            "cab_corner_3_left_group_main",
            "cab_corner_4_left_group_main",
            "sink_left_group_main",
            "counter_1_left_left_group_main",
            "counter_corner_left_group_main",
            "island_left_group_main",
            "bottom_left_group_base_main",
            "bottom_left_group_1_main",
            "bottom_left_group_2_main",
            "oven_left_group_main",
            "oven_housing_left_group_main",
            "microwave_left_group_main",
            "micro_housing_left_group_main",
            "top_left_group_main",
            "stack_1_left_group_main",
            "dishwasher_left_group_main",
            "stack_2_left_group_base_main",
            "stack_2_left_group_1_main",
            "stack_2_left_group_2_main",
            "stack_3_left_group_base_main",
            "stack_3_left_group_1_main",
            "stack_3_left_group_2_main",
            "stack_3_left_group_3_main",
            "coffee_machine_left_group_main",
            "paper_towel_left_group_main",
            "knife_block_left_group_main",
            "plant_left_group_main",
            "stool_1_stool_group_main",
            "stool_2_stool_group_main",
            "stool_3_stool_group_main",
        ]
        for body_name in body_names:
            fill_body_occupancy(body_name)

        return grid
    
    def get_grid_map(self, grid_size: float = RobotConfig.GRID_SIZE) -> np.ndarray:
        """Get grid map of the environment.

        Args:
            grid_size: Grid cell size in meters (default: RobotConfig.GRID_SIZE)

        Returns:
            np.ndarray: Binary occupancy grid (0=free, 1=occupied)
        """
        if self.grid_map is None:
            self.grid_map = self._make_grid_map(grid_size)
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
