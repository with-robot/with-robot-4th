# Mobile Manipulator Robot Control API Documentation

## Introduction

This documentation provides instructions for controlling a Panda-Omron mobile manipulator in a MuJoCo simulation environment.

User-submitted code executes in a **restricted sandbox environment** with access to specific control functions.

**Important: Pre-loaded Modules**
- **DO NOT use `import` statements** - they are not allowed in the sandbox environment
- **`time` module is pre-loaded** - use `time.sleep()` directly without importing
- **`math` module is pre-loaded** - use `math.sin()`, `math.cos()`, etc. directly without importing
- **`PI` constant is available** - use `PI` directly (equivalent to `math.pi`)

## System Architecture

### Execution Environment
- **Simulator**: MuJoCo physics engine
- **Sandbox**: Restricted Python environment with limited builtins
- **Execution**: Code executes synchronously and completely before returning

### Coordinate System
- **World Frame**: All positions and orientations are in world coordinates
- **Mobile Base**: Ground plane (x, y, theta)
  - x: forward/backward position in meters (world frame)
  - y: left/right position in meters (world frame)
  - theta: rotation angle in radians (counterclockwise positive)
- **Arm Joints**: 7-DOF Panda arm with joint angles in radians [j1, j2, j3, j4, j5, j6, j7]
- **End Effector**: Position [x, y, z] in meters (world frame), orientation [roll, pitch, yaw] in radians (world frame)
- **Objects**: All object positions and orientations are in world frame

## Available Functions

### Mobile Base Control

#### `get_mobile_position()`

Gets the current mobile base position [x, y, theta].

**Parameters:**
- None

**Returns:**
- `list[float]`: Current position [x, y, theta]
  - x (float): Current x position in meters
  - y (float): Current y position in meters
  - theta (float): Current orientation in radians

**Example:**
```python
# Get current base position
pos = get_mobile_position()
print(f"Base at x={pos[0]}, y={pos[1]}, theta={pos[2]}")

# Move relative to current position
current = get_mobile_position()
new_pos = [current[0] + 1.0, current[1], current[2]]
set_mobile_target_position(new_pos)
```

#### `set_mobile_target_position(mobile_target_position, timeout=10.0, verbose=False)`

Sets the mobile base target position [x, y, theta] and optionally waits for convergence.

**Parameters:**
- `mobile_target_position` (list[float]): Target position [x, y, theta]
  - x (float): Target x position in meters
  - y (float): Target y position in meters
  - theta (float): Target orientation in radians
- `timeout` (float, optional): Maximum wait time in seconds (default: 10.0)
  - Set to 0 for non-blocking behavior (returns immediately)
  - Set to positive value to wait for convergence
- `verbose` (bool, optional): Print convergence progress (default: False)

**Returns:**
- `bool` or `None`: True if converged, False if timeout (when timeout > 0)

**Example:**
```python
# Move to position (1.0, 0.5, 0) and wait for arrival
set_mobile_target_position([1.0, 0.5, 0])

# Move to position with verbose output
set_mobile_target_position([2.0, -0.5, PI/2], verbose=True)

# Non-blocking update (returns immediately)
set_mobile_target_position([0.0, 0.0, 0.0], timeout=0)
```

#### `plan_mobile_path(target_joint, grid_size=0.1)`

Plans a collision-free path for the mobile base to reach a target position using the A* algorithm.

**Parameters:**
- `target_joint` (list[float]): Target position [x, y] in world coordinates
  - x (float): Target x position in meters
  - y (float): Target y position in meters
  - Note: Theta is automatically calculated based on path direction
- `grid_size` (float, optional): Grid cell size in meters (default: 0.1)

**Returns:**
- `list[list[float]]` or `None`: List of waypoints [(x, y, theta), ...] in world coordinates
  - Returns `None` if no path is found or target is unreachable
  - Each waypoint is [x, y, theta] where theta points toward the next waypoint

**Example:**
```python
# Plan path to target location
path = plan_mobile_path([2.0, -3.0])
if path is not None:
    print(f"Found path with {len(path)} waypoints")
    # Execute path by visiting each waypoint
    for waypoint in path:
        set_mobile_target_position(waypoint, verbose=True)
else:
    print("No path found to target")

# Plan with custom grid size
path = plan_mobile_path([1.5, -2.5], grid_size=0.05)

# Check if target is reachable before planning
target = [2.5, -2.8]
path = plan_mobile_path(target)
if path:
    print(f"Target is reachable via {len(path)} waypoints")
```

#### `get_grid_map()`

Gets the binary occupancy grid map of the environment.

**Parameters:**
- None

**Returns:**
- `list[list[int]]`: 2D binary occupancy grid
  - 0 = free space (navigable)
  - 1 = occupied space (obstacle)
  - Grid cell size is 0.1m by default
  - Coordinates: grid[i][j] where i is row (y-axis), j is column (x-axis)

**Example:**
```python
# Get environment map
grid = get_grid_map()
print(f"Grid size: {len(grid)} x {len(grid[0])}")

# Check if a specific cell is free
row, col = 50, 50
if grid[row][col] == 0:
    print(f"Cell ({row}, {col}) is free")
else:
    print(f"Cell ({row}, {col}) is occupied")

# Count free cells
free_cells = sum(row.count(0) for row in grid)
total_cells = len(grid) * len(grid[0])
print(f"Free space: {free_cells}/{total_cells} cells")
```

#### `follow_mobile_path(path_world, timeout_per_waypoint=30.0, verbose=False)`

Follows a planned path by sequentially moving to each waypoint.

**Parameters:**
- `path_world` (list[list[float]]): List of waypoints [(x, y, theta), ...] in world coordinates
  - Typically obtained from `plan_mobile_path()`
- `timeout_per_waypoint` (float, optional): Maximum wait time per waypoint in seconds (default: 30.0)
- `verbose` (bool, optional): Print progress information (default: False)

**Returns:**
- `bool`: True if all waypoints reached successfully, False if any timeout occurred

**Example:**
```python
# Plan and follow a path
path = plan_mobile_path([2.0, -3.0])
if path:
    success = follow_mobile_path(path, verbose=True)
    if success:
        print("Reached destination!")
    else:
        print("Failed to complete path")

# Follow with custom timeout
path = plan_mobile_path([3.0, -2.5])
if path:
    follow_mobile_path(path, timeout_per_waypoint=15.0, verbose=True)
```

### Arm Control

#### `get_arm_joint_position()`

Gets the current arm joint positions [j1~j7].

**Parameters:**
- None

**Returns:**
- `list[float]`: Current joint positions [j1, j2, j3, j4, j5, j6, j7]
  - All values in radians
  - 7 joint angles for the Panda arm

**Example:**
```python
# Get current arm configuration
joints = get_arm_joint_position()
print(f"Current joints: {joints}")

# Move relative to current position (lift joint 5)
current = get_arm_joint_position()
new_config = current.copy()
new_config[4] += 0.1  # Lift joint 5 by 0.1 rad
set_arm_target_joint(new_config)
```

#### `set_arm_target_joint(arm_target_position, timeout=10.0, verbose=False)`

Sets the arm target joint positions [j1~j7] and optionally waits for convergence.

**Parameters:**
- `arm_target_position` (list[float]): Target joint positions [j1, j2, j3, j4, j5, j6, j7]
  - All values in radians
  - 7 joint angles for the Panda arm
- `timeout` (float, optional): Maximum wait time in seconds (default: 10.0)
  - Set to 0 for non-blocking behavior (returns immediately)
  - Set to positive value to wait for convergence
- `verbose` (bool, optional): Print convergence progress (default: False)

**Returns:**
- `bool` or `None`: True if converged, False if timeout (when timeout > 0)

**Example:**
```python
# Move arm to home configuration and wait
home_position = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
set_arm_target_joint(home_position)

# Move with verbose output
set_arm_target_joint([0, 0, 0, -PI/2, 0, PI/2, PI/4], verbose=True)

# Non-blocking update
set_arm_target_joint([0.1, -0.5, 0.2, -1.8, 0.3, 1.2, 0.9], timeout=0)
```

### End Effector Control

#### `get_ee_position()`

Gets the current end effector pose (position and orientation) in world frame.

**Parameters:**
- None

**Returns:**
- `tuple`: (position, orientation) where:
  - `position` (list[float]): [x, y, z] in meters (world frame)
  - `orientation` (list[float]): [roll, pitch, yaw] in radians (XYZ Euler angles, world frame)

**Example:**
```python
# Get end effector pose
pos, ori = get_ee_position()
print(f"EE position: {pos}")
print(f"EE orientation: {ori}")

# Check if end effector is at target height
current_pos, current_ori = get_ee_position()
if current_pos[2] > 0.5:
    print("End effector is high enough")
```

#### `set_ee_target_position(target_pos, timeout=10.0, verbose=False)`

Sets the end effector target position in world frame (position only, no orientation control).

**Parameters:**
- `target_pos` (list[float]): Target position [x, y, z] in meters (world frame)
- `timeout` (float, optional): Maximum wait time in seconds (default: 10.0)
  - Set to 0 for non-blocking behavior
  - Set to positive value to wait for convergence
- `verbose` (bool, optional): Print convergence progress (default: False)

**Returns:**
- `bool`: True if IK succeeded and converged, False if IK failed or timeout

**Example:**
```python
# Move end effector to absolute world position
set_ee_target_position([1.5, -3.0, 1.2])

# Move to position with verbose output
set_ee_target_position([2.0, -2.5, 1.0], verbose=True)

# Non-blocking positioning
set_ee_target_position([1.0, -3.5, 1.3], timeout=0)

# Check success
success = set_ee_target_position([1.2, -3.2, 1.1], timeout=5)
if not success:
    print("Failed to reach target position")
```

### Gripper Control

#### `get_gripper_width()`

Gets the current gripper width in meters.

**Parameters:**
- None

**Returns:**
- `float`: Current gripper width in meters

**Example:**
```python
# Get current gripper width
width = get_gripper_width()
print(f"Gripper width: {width} meters")

# Check if gripper is open
if width > 0.06:
    print("Gripper is open")
else:
    print("Gripper is closed")
```

#### `set_target_gripper_width(target_width, timeout=10.0, verbose=False)`

Sets the gripper target width and optionally waits for convergence.

**Parameters:**
- `target_width` (float): Target gripper width in meters
  - Typical range: 0.0 (closed) to 0.08 (fully open)
- `timeout` (float, optional): Maximum wait time in seconds (default: 10.0)
  - Set to 0 for non-blocking behavior (returns immediately)
  - Set to positive value to wait for convergence
- `verbose` (bool, optional): Print convergence progress (default: False)

**Returns:**
- `bool`: True if converged, False if timeout (when timeout > 0)

**Example:**
```python
# Open gripper fully
set_target_gripper_width(0.08)

# Close gripper
set_target_gripper_width(0.0)

# Partial grip (e.g., for grasping)
set_target_gripper_width(0.04, verbose=True)

# Non-blocking gripper control
set_target_gripper_width(0.06, timeout=0)

# Check success
success = set_target_gripper_width(0.02, timeout=5)
if not success:
    print("Failed to close gripper")
```

### Pick & Place Operations

#### `pick_object(object_pos, approach_height=0.1, lift_height=0.2, return_to_home=True, timeout=10.0, verbose=False)`

Picks up an object at the specified position using a 6-step sequence.

**Parameters:**
- `object_pos` (list[float]): Object position [x, y, z] in world coordinates
- `approach_height` (float, optional): Height above object to approach before grasping (default: 0.1m)
- `lift_height` (float, optional): Height to lift object after grasping (default: 0.2m)
- `return_to_home` (bool, optional): Whether to return arm to home position after picking (default: True)
- `timeout` (float, optional): Maximum wait time per motion in seconds (default: 10.0)
- `verbose` (bool, optional): Print progress information (default: False)

**Returns:**
- `bool`: True if pick succeeded, False if any step failed

**Example:**
```python
# Get object position
objects = get_object_positions()
if 'object_apple_0' in objects:
    apple_pos = objects['object_apple_0']['pos']
    
    # Pick with default settings
    success = pick_object(apple_pos, verbose=True)
    if success:
        print("Successfully picked up apple!")

# Pick with custom heights
success = pick_object(
    object_pos=[2.0, -3.0, 0.8],
    approach_height=0.15,
    lift_height=0.25,
    return_to_home=False,
    verbose=True
)
```

#### `place_object(place_pos, approach_height=0.2, retract_height=0.3, return_to_home=True, timeout=10.0, verbose=False)`

Places an object at the specified position using a 4-step sequence.

**Parameters:**
- `place_pos` (list[float]): Target placement position [x, y, z] in world coordinates
- `approach_height` (float, optional): Height above placement to approach before lowering (default: 0.2m)
- `retract_height` (float, optional): Height to retract after releasing (default: 0.3m)
- `return_to_home` (bool, optional): Whether to return arm to home position after placing (default: True)
- `timeout` (float, optional): Maximum wait time per motion in seconds (default: 10.0)
- `verbose` (bool, optional): Print progress information (default: False)

**Returns:**
- `bool`: True if place succeeded, False if any step failed

**Example:**
```python
# Place with default settings
success = place_object([2.5, -2.8, 0.9], verbose=True)

# Complete pick and place
objects = get_object_positions()
if 'object_banana_1' in objects:
    banana_pos = objects['object_banana_1']['pos']
    
    # Pick
    if pick_object(banana_pos, verbose=True):
        # Place at new location
        place_object(
            place_pos=[3.0, -2.0, 0.9],
            approach_height=0.25,
            retract_height=0.35,
            verbose=True
        )
```

### Object Perception

#### `get_object_positions()`

Gets the positions and orientations of all objects in the scene (bodies starting with 'object_').

**Parameters:**
- None

**Returns:**
- `dict`: Dictionary mapping object names to their properties:
  - Key: object name (str)
  - Value: dict with:
    - `'id'` (int): Body ID in MuJoCo model
    - `'pos'` (list[float]): Position [x, y, z] in meters (world frame)
    - `'ori'` (list[float]): Orientation [roll, pitch, yaw] in radians (XYZ Euler angles, world frame)

**Example:**
```python
# Get all object positions
objects = get_object_positions()

# Access specific object
if 'object_apple_0' in objects:
    apple = objects['object_apple_0']
    print(f"Apple at: {apple['pos']}")
    print(f"Apple orientation: {apple['ori']}")

# Find closest object to a position
target = [2.0, -3.0, 1.0]
closest_name = None
closest_dist = float('inf')

for name, obj in objects.items():
    dx = obj['pos'][0] - target[0]
    dy = obj['pos'][1] - target[1]
    dz = obj['pos'][2] - target[2]
    dist = (dx**2 + dy**2 + dz**2) ** 0.5
    if dist < closest_dist:
        closest_dist = dist
        closest_name = name

print(f"Closest object: {closest_name} at distance {closest_dist}")

# Move end effector to object position
if 'object_banana_1' in objects:
    banana_pos = objects['object_banana_1']['pos']
    # Move 10cm above the banana
    target_pos = [banana_pos[0], banana_pos[1], banana_pos[2] + 0.1]
    set_ee_target_position(target_pos)
```

## Available Constants and Utilities

### Constants
- `PI`: Mathematical constant π (numpy.pi ≈ 3.14159265359)
- `RESULT`: Empty dictionary for storing return values from user code

### Builtin Functions and Modules
The sandbox provides access to the following Python builtins and pre-loaded modules:

**Builtins:**
- `print(*args)`: Output text for debugging
- `range(start, stop, step)`: Generate numeric sequences
- `float(x)`: Convert to floating-point number

**IMPORTANT: Import statements are NOT allowed in the sandbox!**

## Important Notes

- **End Effector**: Position-only control (orientation cannot be set)
- **Sandbox**: No file system or network access
- **Execution**: All code runs synchronously