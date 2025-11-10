"""Sandboxed code execution layer for mobile base control with waiting logic."""

import time
import numpy as np
from simulator import MujocoSimulator

# Simulator instance injected by main.py at startup
simulator: MujocoSimulator = None


def _wait_for_convergence(get_pos_diff_fn, get_vel_fn, pos_threshold, vel_threshold, 
                          timeout=10.0, stable_frames=5, verbose=False):
    """
    Wait until position and velocity converge with stability check.
    
    Args:
        get_pos_diff_fn: Function returning position error
        get_vel_fn: Function returning velocity
        pos_threshold: Position error threshold for convergence
        vel_threshold: Velocity norm threshold for convergence
        timeout: Maximum wait time in seconds
        stable_frames: Number of consecutive frames within threshold
        verbose: Print convergence progress
    
    Returns:
        bool: True if converged, False if timeout
    """
    start_time = time.time()
    stable_count = 0
    iterations = 0
    
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


def set_mobile_target_position(mobile_target_position, timeout=10.0, verbose=False):
    """
    Set mobile base target position [x, y, theta] in meters and radians.

    Args:
        mobile_target_position: mobile base target position [x, y, theta]
        timeout: Maximum wait time in seconds (default: 10s)
        verbose: Print convergence progress
    """
    # Update mobile base target position immediately (non-blocking)
    simulator.set_mobile_target_position(mobile_target_position)

    if timeout > 0:
        def get_mobile_pos_diff_weighted():
            diff = simulator.get_mobile_position_diff()
            diff[-1] /= 2  # Theta weighted at 50%
            return diff
        
        return _wait_for_convergence(
            get_mobile_pos_diff_weighted,
            simulator.get_current_mobile_velocity,
            pos_threshold=0.1,
            vel_threshold=0.05,  # ~0.05 m/s or rad/s
            timeout=timeout,
            stable_frames=5,
            verbose=verbose
        )


def set_arm_target_position(arm_target_position, timeout=10.0, verbose=False):
    """
    Set arm target joint positions [j1~j7] in radians.

    Args:
        arm_target_position: arm target joint positions [j1~j7] in radians
        timeout: Maximum wait time in seconds (default: 10s)
        verbose: Print convergence progress
    """
    # Update arm target position immediately (non-blocking)
    simulator.set_arm_target_position(arm_target_position)

    if timeout > 0:
        return _wait_for_convergence(
            simulator.get_arm_position_diff,
            simulator.get_current_arm_velocity,
            pos_threshold=0.1,
            vel_threshold=0.1,  # ~0.1 rad/s
            timeout=timeout,
            stable_frames=5,
            verbose=verbose
        )


def exec_code(code):
    """
    Execute user code in sandboxed environment with mobile base control access.

    Args:
        code: Python code string to execute

    Available in sandbox:
        - Builtins: print, range, float, time
        - Constants: PI (numpy.pi)
        - Functions:
            - set_mobile_target_position(mobile_target_position, wait=True) - Mobile base control
            - set_arm_target_position(arm_target_position, wait=True) - Arm control
    """
    # Define sandboxed environment with limited access
    safe_globals = {
        "__builtins__": {"print": print, "range": range, "float": float, "time": time},
        "PI": np.pi,
        "set_mobile_target_position": set_mobile_target_position,
        "set_arm_target_position": set_arm_target_position,
    }
    exec(code, safe_globals)
