"""Sandboxed code execution layer for robot control with waiting logic."""

import time
import numpy as np
from simulator import MujocoSimulator
from obstacle_manager import obstacle_manager

# Simulator instance injected by main.py at startup
simulator: MujocoSimulator = None

# Optional voice feedback callback
voice_callback = None


def set_target_position(x, y, theta, wait=True):
    """
    Set target position [x, y, theta] in meters and radians.

    Args:
        x: Target x position in meters
        y: Target y position in meters
        theta: Target orientation in radians
        wait: If True, blocks until reached (max 10s, error < 0.1m)
    """
    # Check for obstacles at target position
    obstacle = obstacle_manager.check_path(x, y)

    if obstacle:
        # Obstacle detected! Replan path
        current_pos = simulator.get_current_position()
        print(f"🚧 장애물 발견! 경로를 재계획합니다...")

        # Trigger voice feedback if available
        if voice_callback:
            voice_callback("장애물을 발견했습니다. 경로를 재계획하는 중입니다.")

        # Generate detour waypoints
        waypoints = obstacle_manager.suggest_detour(
            current_x=current_pos[0],
            current_y=current_pos[1],
            target_x=x,
            target_y=y,
            obstacle=obstacle
        )

        # Follow detour waypoints
        for i, (wp_x, wp_y) in enumerate(waypoints, 1):
            print(f"  🔄 우회 경로 {i}/{len(waypoints)}: ({wp_x:.2f}, {wp_y:.2f})")
            simulator.set_target_position(wp_x, wp_y, theta)

            if wait:
                # Wait at each waypoint
                _wait_for_position(wp_x, wp_y, theta)

        print(f"✅ 우회 완료! 목표 도달")
        return

    # No obstacle - proceed normally
    simulator.set_target_position(x, y, theta)

    # DEBUG: Log target
    print(f"  → Target: ({x:.2f}, {y:.2f}, {theta:.2f})")

    if wait:
        _wait_for_position(x, y, theta)


def _wait_for_position(x, y, theta):
    """Helper function to wait until robot reaches target position"""
    max_iterations = 100  # 10 seconds
    for i in range(max_iterations):
        # Calculate position error with reduced weight on theta
        position_diff = simulator.get_position_diff()
        position_diff[-1] /= 2  # Theta error weighted at 50%

        error_norm = np.linalg.norm(position_diff)

        # Check if position error is within threshold
        if error_norm < 0.1:
            print(f"  ✓ Reached target (error: {error_norm:.3f})")
            break

        # DEBUG: Log progress every 2 seconds
        if i % 20 == 0 and i > 0:
            current_pos = simulator.get_current_position()
            print(f"  ... waiting (error: {error_norm:.3f}, pos: {current_pos[0]:.2f}, {current_pos[1]:.2f})")

        time.sleep(0.1)
    else:
        # Timeout reached
        print(f"  ⚠️ Timeout after {max_iterations/10}s (error: {error_norm:.3f})")
        print(f"     Consider using wait=False for demo")


# Robot Initial Position (Global)
ROBOT_INITIAL_X = 0.8
ROBOT_INITIAL_Y = -3.45

def add_obstacle(x, y, radius=0.3):
    """
    Add a physical obstacle to the environment.
    Args:
        x, y: Position in ROBOT RELATIVE coordinates
    """
    # Add logical obstacle to manager (Relative coords)
    obstacle = obstacle_manager.add_obstacle(x, y, radius)

    # Add physical obstacle to simulator (Global coords)
    if simulator:
        # Convert Relative -> Global
        global_x = x + ROBOT_INITIAL_X
        global_y = y + ROBOT_INITIAL_Y
        
        obs_id = simulator.add_physical_obstacle(global_x, global_y, radius)
        if obs_id is None:
            print("⚠️  Physical obstacle pool full, only using logical obstacle")

    return obstacle


def clear_obstacles():
    """Remove all obstacles from the environment."""
    obstacle_manager.clear_obstacles()

    # Clear physical obstacles from simulator
    if simulator:
        simulator.clear_physical_obstacles()


def get_obstacle_info():
    """Get information about current obstacles."""
    return obstacle_manager.get_obstacle_info()


def get_current_position():
    """Get current robot position [x, y, theta] (Relative)."""
    if simulator:
        return simulator.get_current_position()
    return [0.0, 0.0, 0.0]


def move_agent_b(x, y, theta=0.0):
    """
    Move 'Agent B' (simulated by a physical obstacle).
    Args:
        x, y: Target position in ROBOT RELATIVE coordinates
    """
    if simulator:
        # Use obstacle_1 as Agent B
        obs_id = 1
        
        # Convert Relative -> Global
        global_x = x + ROBOT_INITIAL_X
        global_y = y + ROBOT_INITIAL_Y
        
        # If obstacle doesn't exist, create it
        if obs_id not in simulator.physical_obstacles:
            simulator.add_physical_obstacle(global_x, global_y, radius=0.2)
        
        # Move it
        simulator.set_physical_obstacle_position(obs_id, global_x, global_y, theta)
        print(f"🤖 Agent B moved to ({x:.2f}, {y:.2f}) [Global: {global_x:.2f}, {global_y:.2f}]")


def spiral_search(start_x, start_y, max_radius=2.0, steps=8):
    """
    Scenario 2: Search for object using spiral pattern.
    """
    print(f"🔍 Starting spiral search from ({start_x}, {start_y})...")
    import math
    
    for i in range(steps * 2):
        angle = i * (math.pi / 4)
        radius = 0.5 + (i * 0.2)  # Increasing radius
        
        if radius > max_radius:
            break
            
        x = start_x + radius * math.cos(angle)
        y = start_y + radius * math.sin(angle)
        
        print(f"  👀 Searching at ({x:.2f}, {y:.2f})...")
        set_target_position(x, y, angle, wait=True)
        time.sleep(0.5)
        
    print("✅ Search complete (simulated)")


def get_safe_start_point():
    """
    Get a safe starting position - move significantly east into corridor.
    """
    return 3.0, 2.0, 0.0

def get_safe_target():
    """
    Get a safe target position - travel far along corridor.
    """
    return 3.0, 10.0, 0.0


def cooperative_move(target_x, target_y):
    """
    Scenario 5: Cooperative movement with Agent B.
    Moves both robot and Agent B synchronously to target.
    """
    print(f"🤝 Starting cooperative move to ({target_x}, {target_y})...")
    
    current_pos = simulator.get_current_position()
    start_x, start_y = current_pos[0], current_pos[1]
    
    # Interpolate movement for synchronization
    steps = 10
    for i in range(1, steps + 1):
        t = i / steps
        
        # Robot position
        rx = start_x + (target_x - start_x) * t
        ry = start_y + (target_y - start_y) * t
        
        # Agent B position (maintaining offset)
        bx = rx + 0.5  # Agent B is 0.5m to the right
        by = ry
        
        # Move both
        move_agent_b(bx, by)
        set_target_position(rx, ry, 0, wait=True)
        
    print("✅ Cooperative move complete!")


def exec_code(code):
    """
    Execute user code in sandboxed environment with robot control access.

    Args:
        code: Python code string to execute

    Available in sandbox:
        - Builtins: print, range, float, time
        - Constants: PI (numpy.pi)
        - Functions:
          - set_target_position(x, y, theta, wait=True)
          - add_obstacle(x, y, radius=0.3)
          - clear_obstacles()
          - get_obstacle_info()
    """
    # Define sandboxed environment with limited access
    safe_globals = {
        "__builtins__": {"print": print, "range": range, "float": float, "time": time, "len": len, "abs": abs},
        "PI": np.pi,
        "set_target_position": set_target_position,
        "add_obstacle": add_obstacle,
        "clear_obstacles": clear_obstacles,
        "get_obstacle_info": get_obstacle_info,
        "get_current_position": get_current_position,
        "get_safe_start_point": get_safe_start_point,
        "get_safe_target": get_safe_target,
        "move_agent_b": move_agent_b,
        "spiral_search": spiral_search,
        "cooperative_move": cooperative_move,
    }
    exec(code, safe_globals)
