"""Sandboxed code execution layer for mobile base control with waiting logic."""

import time
import numpy as np
from typing import Optional, List, Tuple, Callable, Dict, Any
from simulator import MujocoSimulator

# Simulator instance injected by main.py at startup
simulator: MujocoSimulator = None


def _wait_for_convergence(
    get_pos_diff_fn: Callable[[], np.ndarray],
    get_vel_fn: Callable[[], np.ndarray],
    pos_threshold: float,
    vel_threshold: float,
    timeout: float = 10.0,
    stable_frames: int = 5,
    verbose: bool = False
) -> bool:
    """Wait for position and velocity convergence with stability check."""
    start_time = time.time()
    stable_count = 0
    iterations = 0
    pos_error = float('inf')
    vel_error = float('inf')

    while time.time() - start_time < timeout:
        pos_diff = get_pos_diff_fn()
        velocity = get_vel_fn()

        pos_error = np.linalg.norm(pos_diff)
        vel_error = np.linalg.norm(velocity)

        # Check both position and velocity convergence
        if pos_error < pos_threshold and vel_error < vel_threshold:
            stable_count += 1
            if stable_count >= stable_frames:
                if verbose:
                    print(f"Converged after {time.time() - start_time:.2f}s ({iterations} iterations)")
                return True
        else:
            stable_count = 0  # Reset if not stable

        iterations += 1

        # Adaptive sleep: longer when far, shorter when close
        if pos_error > pos_threshold * 3:
            time.sleep(0.1)
        elif pos_error > pos_threshold * 1.5:
            time.sleep(0.05)
        else:
            time.sleep(0.02)

    if verbose:
        print(f"Timeout after {timeout}s (pos_error={pos_error:.4f}, vel_error={vel_error:.4f})")
    return False


def get_mobile_joint_position() -> List[float]:
    """Get current mobile base position [x, y, theta]."""
    pos = simulator.get_mobile_joint_position().tolist()
    return pos


def set_mobile_target_joint(
    mobile_target_position: List[float],
    timeout: float = 10.0,
    verbose: bool = False
) -> bool:
    """
    Set mobile base target position [x, y, theta] in meters and radians.

    Args:
        mobile_target_position: mobile base target position [x, y, theta]
        timeout: Maximum wait time in seconds (default: 60s)
        verbose: Print convergence progress
    """
    # Update mobile base target position immediately (non-blocking)
    simulator.set_mobile_target_joint(mobile_target_position)

    success = True
    if success and timeout > 0:
        def get_mobile_pos_diff_weighted() -> np.ndarray:
            diff = simulator.get_mobile_joint_diff()
            diff[-1] /= 2  # Theta weighted at 50%
            return diff

        converged = _wait_for_convergence(
            get_mobile_pos_diff_weighted,
            simulator.get_mobile_joint_velocity,
            pos_threshold=0.1,
            vel_threshold=0.05,  # ~0.05 m/s or rad/s
            timeout=timeout,
            stable_frames=5,
            verbose=verbose
        )
        success = converged
    return success


def plan_mobile_path(
    target_joint: List[float],
    grid_size: float = 0.1
) -> Optional[List[List[float]]]:
    """
    Plan path for mobile base to reach target joint using A* algorithm.
    
    Args:
        target_joint: Target position [x, y] in world coordinates
        grid_size: Grid cell size in meters
        
    Returns:
        path: List of waypoints [(x, y, theta), ...] in world coordinates, or None if unreachable
    """
    path = simulator.plan_mobile_path(target_joint, grid_size)
    if path is not None:
        path = [p.tolist() for p in path]
    return path


def get_arm_joint_position() -> List[float]:
    """Get current arm joint positions [j1~j7] in radians."""
    pos = simulator.get_arm_joint_position().tolist()
    return pos


def set_arm_target_joint(
    arm_target_position: List[float],
    timeout: float = 10.0,
    verbose: bool = False
) -> bool:
    """
    Set arm target joint positions [j1~j7] in radians.

    Args:
        arm_target_position: arm target joint positions [j1~j7] in radians
        timeout: Maximum wait time in seconds (default: 10s)
        verbose: Print convergence progress
    """
    # Update arm target position immediately (non-blocking)
    simulator.set_arm_target_joint(arm_target_position)

    success = True
    if success and timeout > 0:
        converged = _wait_for_convergence(
            simulator.get_arm_joint_diff,
            simulator.get_arm_joint_velocity,
            pos_threshold=0.1,
            vel_threshold=0.1,  # ~0.1 rad/s
            timeout=timeout,
            stable_frames=5,
            verbose=verbose
        )
        success = converged
    return success


def get_ee_position() -> Tuple[List[float], List[float]]:
    """Get current end effector pose as tuple: (position, orientation) where position=[x,y,z], orientation=[roll,pitch,yaw]."""
    pos, ori = simulator.get_ee_position()

    return pos.tolist(), ori.tolist()


def set_ee_target_position(
    target_pos: List[float],
    timeout: float = 10.0,
    verbose: bool = False
) -> bool:
    """
    Set end effector target position in world frame.

    Args:
        target_pos: [x, y, z] target position in meters
        timeout: Maximum wait time in seconds (default: 10.0)
        verbose: Print convergence progress

    Returns:
        bool: True if converged, False if timeout
    """
    success, joint_angles = simulator.set_ee_target_position(target_pos)

    if success and timeout > 0:
        converged = _wait_for_convergence(
            simulator.get_arm_joint_diff,
            simulator.get_arm_joint_velocity,
            pos_threshold=0.1,
            vel_threshold=0.1,
            timeout=timeout,
            stable_frames=5,
            verbose=verbose
        )
        success = converged
    return success


def get_gripper_width() -> float:
    """Get current gripper width in meters."""
    return simulator.get_gripper_width()


def set_target_gripper_width(
    target_width: float,
    timeout: float = 10.0,
    verbose: bool = False
) -> bool:
    """
    Set gripper target width.

    Args:
        target_width: target gripper width in meters
        timeout: Maximum wait time in seconds (default: 10.0)
        verbose: Print convergence progress

    Returns:
        bool: True if converged, False if timeout
    """
    simulator.set_target_gripper_width(target_width)

    success = True
    if success and timeout > 0:
        converged = _wait_for_convergence(
            simulator.get_gripper_width_diff,
            simulator.get_gripper_width_velocity,
            pos_threshold=0.02,
            vel_threshold=0.02,
            timeout=timeout,
            stable_frames=5,
            verbose=verbose
        )
        time.sleep(1.0)  # Wait for gripper to reach target width
        success = converged
    return success


def get_grid_map() -> List[List[int]]:
    """Get grid map of the environment."""
    return simulator.get_grid_map().tolist()


def get_object_positions() -> Dict[str, Dict[str, Any]]:
    """Get list of object dictionaries with id, name, position and orientation."""
    objects = simulator.get_object_positions()
    for obj in objects.values():
        obj['pos'] = obj['pos'].tolist()
        obj['ori'] = obj['ori'].tolist()
    return objects


def exec_code(code: str) -> Optional[Dict[str, Any]]:
    """
    Execute user code in sandboxed environment.

    Available functions:
        - get_mobile_joint_position() -> [x, y, theta]
        - set_mobile_target_joint(mobile_target_position, timeout, verbose)
        - plan_mobile_path(target_joint, grid_size)
        - get_arm_joint_position() -> [j1~j7]
        - set_arm_target_joint(arm_target_position, timeout, verbose)
        - get_ee_position() -> (position, orientation) where position=[x,y,z], orientation=[roll,pitch,yaw]
        - set_ee_target_position(target_pos, timeout, verbose)
        - set_target_gripper_width(target_width, timeout, verbose)
        - get_grid_map() -> grid map of the environment
        - get_object_positions() -> list of object dictionaries with id, name, position and orientation
    """
    # Define sandboxed environment with limited access
    safe_globals = {
        "__builtins__": {"print": print, "range": range, "float": float, "time": time},
        "PI": np.pi,
        "RESULT": {},
        "get_mobile_joint_position": get_mobile_joint_position,
        "set_mobile_target_joint": set_mobile_target_joint,
        "plan_mobile_path": plan_mobile_path,
        "get_arm_joint_position": get_arm_joint_position,
        "set_arm_target_joint": set_arm_target_joint,
        "get_ee_position": get_ee_position,
        "set_ee_target_position": set_ee_target_position,
        "set_target_gripper_width": set_target_gripper_width,
        "get_grid_map": get_grid_map,
        "get_object_positions": get_object_positions,
    }
    exec(code, safe_globals)
    return safe_globals.get("RESULT")
