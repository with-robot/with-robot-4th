"""Sandboxed code execution layer for mobile base control with waiting logic."""

import time
import numpy as np
from simulator import MujocoSimulator

# Simulator instance injected by main.py at startup
simulator: MujocoSimulator = None


def _wait_for_convergence(get_pos_diff_fn, get_vel_fn, pos_threshold, vel_threshold,
                          timeout=10.0, stable_frames=5, verbose=False):
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


def get_mobile_joint_position():
    """Get current mobile base position [x, y, theta]."""
    pos = simulator.get_mobile_joint_position().tolist()
    return pos


def set_mobile_target_joint(mobile_target_position, timeout=10.0, verbose=False):
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
        def get_mobile_pos_diff_weighted():
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


def get_arm_joint_position():
    """Get current arm joint positions [j1~j7] in radians."""
    pos = simulator.get_arm_joint_position().tolist()
    return pos


def set_arm_target_joint(arm_target_position, timeout=10.0, verbose=False):
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


def get_ee_position():
    """Get current end effector pose as tuple: (position, orientation) where position=[x,y,z], orientation=[roll,pitch,yaw]."""
    pos, ori = simulator.get_ee_position()

    return pos.tolist(), ori.tolist()


def set_ee_target_position(target_pos, timeout=10.0, verbose=False):
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


def get_object_positions():
    """Get list of object dictionaries with id, name, position and orientation."""
    return simulator.get_object_positions()


def exec_code(code):
    """
    Execute user code in sandboxed environment.

    Available functions:
        - get_mobile_joint_position() -> [x, y, theta]
        - set_mobile_target_joint(mobile_target_position, timeout, verbose)
        - get_arm_joint_position() -> [j1~j7]
        - set_arm_target_joint(arm_target_position, timeout, verbose)
        - get_ee_position() -> (position, orientation) where position=[x,y,z], orientation=[roll,pitch,yaw]
        - set_ee_target_position(target_pos, timeout, verbose)
        - get_object_positions() -> list of object dictionaries with id, name, position and orientation
    """
    # Define sandboxed environment with limited access
    safe_globals = {
        "__builtins__": {"print": print, "range": range, "float": float, "time": time},
        "PI": np.pi,
        "RESULT": {},
        "get_mobile_joint_position": get_mobile_joint_position,
        "set_mobile_target_joint": set_mobile_target_joint,
        "get_arm_joint_position": get_arm_joint_position,
        "set_arm_target_joint": set_arm_target_joint,
        "get_ee_position": get_ee_position,
        "set_ee_target_position": set_ee_target_position,
        "get_object_positions": get_object_positions,
    }
    exec(code, safe_globals)
    return safe_globals.get("RESULT")
