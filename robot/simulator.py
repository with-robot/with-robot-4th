"""MuJoCo robot simulator with automatic position control for Panda-Omron mobile manipulator."""

import time
import numpy as np
import mujoco, mujoco.viewer


class RobotConfig:
    """Robot simulation configuration constants."""

    # Mobile base joints: [x, y, theta]
    JOINT_NAMES = [
        "mobilebase0_joint_mobile_side",
        "mobilebase0_joint_mobile_forward",
        "mobilebase0_joint_mobile_yaw"
    ]

    ACTUATOR_NAMES = [
        "mobilebase0_actuator_mobile_side",
        "mobilebase0_actuator_mobile_forward",
        "mobilebase0_actuator_mobile_yaw"
    ]

    # PD controller gains: [kp_x, kp_y, kp_theta]
    # Reduced to fit within actuator ctrlrange [-1.0, 1.0] for x/y
    KP = np.array([0.8, 0.8, 0.6])
    KD = np.array([0.4, 0.4, 0.25])

    # Camera settings
    CAM_LOOKAT = [2.15, -0.8, 0.8]
    CAM_DISTANCE = 5.0
    CAM_AZIMUTH = 135
    CAM_ELEVATION = -25

    INITIAL_POSITION = np.array([0.0, 0.0, 0.0])


import random

class MujocoSimulator:
    """MuJoCo simulator with PD-controlled mobile base position tracking."""

    def __init__(self, xml_path="../model/robocasa/panda_omron.xml"):
        """Initialize simulator with MuJoCo model and control indices."""
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data  = mujoco.MjData(self.model)
        self._target_position = RobotConfig.INITIAL_POSITION.copy()

        # Visual obstacle tracking
        self.visual_obstacles = []  # List of (site_id, x, y, radius)

        # Physical obstacle pool (Standard shapes + Asset objects)
        self.physical_obstacles = {}  # {obstacle_id: {'x': x, 'y': y, 'radius': r, 'body_id': id}}
        
        # Pool of available obstacle suffixes (maps to body name "obstacle_{suffix}")
        # 1-5 are cylinders/boxes, others are assets
        self.available_obstacle_ids = [
            '1', '2', '3', '4', '5',
            'apple', 'banana', 'bottled_water', 'can', 'mug'
        ]

        # Resolve joint/actuator names to indices
        self.joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                         for name in RobotConfig.JOINT_NAMES]
        self.actuator_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                            for name in RobotConfig.ACTUATOR_NAMES]

    def get_target_position(self):
        """Get current target position [x, y, theta]."""
        return self._target_position

    def set_target_position(self, x, y, theta):
        """
        Set target position [x, y, theta] in meters and radians.
        """
        self._target_position = np.array([x, y, theta])

    def get_current_position(self):
        """Get current position [x, y, theta] from joint states."""
        return np.array([
            self.data.qpos[self.joint_ids[0]],
            self.data.qpos[self.joint_ids[1]],
            self.data.qpos[self.joint_ids[2]]
        ])
    
    def get_position_diff(self):
        """Get position error [delta_x, delta_y, delta_theta] between target and current position."""
        return self._target_position - self.get_current_position()

    def get_current_velocity(self):
        """Get current velocity [vx, vy, omega] from joint velocities."""
        return np.array([
            self.data.qvel[self.joint_ids[0]],
            self.data.qvel[self.joint_ids[1]],
            self.data.qvel[self.joint_ids[2]]
        ])

    def _compute_control(self):
        """Compute PD control commands [vx, vy, omega] to reach target."""
        current_pos = self.get_current_position()
        current_vel = self.get_current_velocity()

        # 1. Calculate Global Error
        pos_error_global = self._target_position - current_pos
        pos_error_global[2] = np.arctan2(np.sin(pos_error_global[2]), np.cos(pos_error_global[2]))  # Normalize angle

        # 2. Convert to Local Frame (Robot Body Frame)
        # Robot orientation (theta)
        theta = current_pos[2]
        c, s = np.cos(theta), np.sin(theta)
        
        # Rotation matrix R_global_to_local = R_local_to_global^T
        # R_local_to_global = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
        # R_global_to_local = [[c, s, 0], [-s, c, 0], [0, 0, 1]]
        
        R_inv = np.array([
            [c, s, 0],
            [-s, c, 0],
            [0, 0, 1]
        ])
        
        pos_error_local = R_inv @ pos_error_global
        
        # 3. PD Control in Local Frame
        # Target velocity in local frame
        target_vel_local = RobotConfig.KP * pos_error_local - RobotConfig.KD * current_vel
        
        # Apply limits (optional, but good for safety)
        target_vel_local = np.clip(target_vel_local, -5.0, 5.0)

        return target_vel_local

    def add_visual_obstacle(self, x, y, radius=0.3):
        """
        Add a visual obstacle marker in the MuJoCo viewer.

        Args:
            x, y: Position in meters
            radius: Obstacle radius in meters
        """
        # Store obstacle info for rendering
        obstacle_info = {
            'x': x,
            'y': y,
            'radius': radius,
            'height': 0.8  # Visual height for cylinder
        }
        self.visual_obstacles.append(obstacle_info)
        print(f"🎨 Visual obstacle added at ({x:.2f}, {y:.2f})")
        return obstacle_info

    def clear_visual_obstacles(self):
        """Clear all visual obstacles."""
        self.visual_obstacles.clear()
        print("🎨 Visual obstacles cleared")

    def add_physical_obstacle(self, x, y, radius=0.3):
        """
        Add a physical obstacle by activating one from the pool.
        Selects a random available obstacle (shape or asset).

        Args:
            x, y: Position in meters
            radius: Obstacle radius (used for collision detection, default 0.3m)

        Returns:
            obstacle_id if successful, None if pool is exhausted
        """
        if not self.available_obstacle_ids:
            print("⚠️ Obstacle pool exhausted! Maximum physical obstacles reached.")
            return None

        # Get random available obstacle ID
        obs_id = random.choice(self.available_obstacle_ids)
        self.available_obstacle_ids.remove(obs_id)
        
        body_name = f"obstacle_{obs_id}"

        # Get body ID
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)

        # Find the joint for this obstacle (freejoint)
        joint_id = None
        for i in range(self.model.njnt):
            if self.model.jnt_bodyid[i] == body_id:
                joint_id = i
                break

        if joint_id is None:
            print(f"❌ Could not find joint for {body_name}")
            self.available_obstacle_ids.append(obs_id)
            return None

        # Set obstacle position (freejoint: 7 DOF - qpos[qpos_adr:qpos_adr+7])
        # [x, y, z, qw, qx, qy, qz] for freejoint
        qpos_adr = self.model.jnt_qposadr[joint_id]
        self.data.qpos[qpos_adr] = x      # x position
        self.data.qpos[qpos_adr + 1] = y  # y position
        self.data.qpos[qpos_adr + 2] = 0.0  # z position (on ground level)
        # Quaternion for no rotation: [1, 0, 0, 0]
        self.data.qpos[qpos_adr + 3] = 1.0  # qw
        self.data.qpos[qpos_adr + 4] = 0.0  # qx
        self.data.qpos[qpos_adr + 5] = 0.0  # qy
        self.data.qpos[qpos_adr + 6] = 0.0  # qz

        # Zero velocity
        qvel_adr = self.model.jnt_dofadr[joint_id]
        self.data.qvel[qvel_adr:qvel_adr+6] = 0.0

        # Forward dynamics to settle the object
        mujoco.mj_forward(self.model, self.data)

        # Store obstacle info
        self.physical_obstacles[obs_id] = {
            'x': x,
            'y': y,
            'radius': radius,
            'body_id': body_id,
            'joint_id': joint_id
        }

        print(f"🏗️ Physical obstacle_{obs_id} added at ({x:.2f}, {y:.2f})")
        return obs_id

    def remove_physical_obstacle(self, obs_id):
        """
        Remove a physical obstacle by moving it off-screen.

        Args:
            obs_id: Obstacle ID to remove
        """
        if obs_id not in self.physical_obstacles:
            print(f"⚠️ Obstacle {obs_id} not found")
            return False

        obs_info = self.physical_obstacles[obs_id]
        joint_id = obs_info['joint_id']
        qpos_adr = self.model.jnt_qposadr[joint_id]

        # Move obstacle far away
        self.data.qpos[qpos_adr] = -100  # x
        self.data.qpos[qpos_adr + 1] = -100  # y

        # Remove from active obstacles
        del self.physical_obstacles[obs_id]
        self.available_obstacle_ids.append(obs_id)
        # self.available_obstacle_ids.sort() # Don't sort to keep randomness or mixed types

        print(f"🗑️ Physical obstacle_{obs_id} removed")
        return True

    def clear_physical_obstacles(self):
        """Clear all physical obstacles."""
        for obs_id in list(self.physical_obstacles.keys()):
            self.remove_physical_obstacle(obs_id)
        print("🗑️ All physical obstacles cleared")

    def set_physical_obstacle_position(self, obs_id, x, y, theta=0.0):
        """
        Set position of a physical obstacle (used to simulate Agent B).
        
        Args:
            obs_id: Obstacle ID
            x, y: Target position
            theta: Target orientation (yaw)
        """
        if obs_id not in self.physical_obstacles:
            return False
            
        obs_info = self.physical_obstacles[obs_id]
        joint_id = obs_info['joint_id']
        qpos_adr = self.model.jnt_qposadr[joint_id]
        
        # Update position
        self.data.qpos[qpos_adr] = x
        self.data.qpos[qpos_adr + 1] = y
        
        # Update orientation (convert theta to quaternion)
        # Axis: (0, 0, 1), Angle: theta
        # q = [cos(theta/2), 0, 0, sin(theta/2)]
        half_theta = theta / 2
        self.data.qpos[qpos_adr + 3] = np.cos(half_theta)
        self.data.qpos[qpos_adr + 4] = 0.0
        self.data.qpos[qpos_adr + 5] = 0.0
        self.data.qpos[qpos_adr + 6] = np.sin(half_theta)
        
        # Zero velocity to prevent physics glitches
        qvel_adr = self.model.jnt_dofadr[joint_id]
        self.data.qvel[qvel_adr:qvel_adr+6] = 0.0
        
        # Update internal state
        obs_info['x'] = x
        obs_info['y'] = y
        
        return True

    def _render_obstacles(self, viewer):
        """Render visual obstacles as cylinders in the viewer."""
        for obs in self.visual_obstacles:
            # Create cylinder visualization
            # Position: [x, y, z] where z is half the height
            pos = np.array([obs['x'], obs['y'], obs['height']/2], dtype=np.float64)
            # Size: [radius, half_height, 0] - MuJoCo requires 3 elements for cylinder
            size = np.array([obs['radius'], obs['height']/2, 0.0], dtype=np.float64)
            # Color: Red with transparency - must be float32
            rgba = np.array([1.0, 0.2, 0.2, 0.6], dtype=np.float32)

            # Draw cylinder using MuJoCo viewer API
            mujoco.mjv_initGeom(
                viewer.user_scn.geoms[viewer.user_scn.ngeom],
                mujoco.mjtGeom.mjGEOM_CYLINDER,
                size,
                pos,
                np.eye(3).flatten(),
                rgba
            )
            viewer.user_scn.ngeom += 1

            # Prevent overflow
            if viewer.user_scn.ngeom >= viewer.user_scn.maxgeom:
                break

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
            v.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = True  # Enable transparency for obstacles
            v.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
            v.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = False
            v.opt.flags[mujoco.mjtVisFlag.mjVIS_PERTFORCE] = False
            v.opt.flags[mujoco.mjtVisFlag.mjVIS_PERTOBJ] = False
            v.opt.frame = mujoco.mjtFrame.mjFRAME_NONE
            v.opt.label = mujoco.mjtLabel.mjLABEL_NONE

            # Main loop
            step_count = 0
            while v.is_running():
                # Render visual obstacles
                v.user_scn.ngeom = 0  # Reset custom geometry count
                self._render_obstacles(v)

                # PD control
                control = self._compute_control()
                self.data.ctrl[self.actuator_ids[0]] = control[0]
                self.data.ctrl[self.actuator_ids[1]] = control[1]
                self.data.ctrl[self.actuator_ids[2]] = control[2]
                
                # Debug output every 100 steps (~5 seconds at 60fps)
                step_count += 1
                if step_count % 100 == 0:
                    pos = self.get_current_position()
                    target = self.get_target_position()
                    print(f"[SIM] pos=({pos[0]:.2f},{pos[1]:.2f}) target=({target[0]:.2f},{target[1]:.2f}) ctrl=[{control[0]:.2f},{control[1]:.2f},{control[2]:.2f}]")
                
                mujoco.mj_step(self.model, self.data)
                v.sync()
