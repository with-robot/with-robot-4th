# Mobile Manipulator Robot Control API Documentation

## Table of Contents
1. [Introduction](#introduction)
2. [System Architecture](#system-architecture)
3. [Available Functions](#available-functions)
   - [Mobile Base Control](#mobile-base-control)
   - [Arm Control](#arm-control)
   - [End Effector Control](#end-effector-control)
4. [Available Constants and Utilities](#available-constants-and-utilities)
5. [Usage Examples](#usage-examples)
6. [Parameters and Safety Information](#parameters-and-safety-information)

## Introduction

This documentation provides detailed instructions for controlling a Panda-Omron mobile manipulator in a MuJoCo simulation environment. The robot operates in a kitchen scene and can be controlled through sandboxed Python code execution via a REST API.

User-submitted code executes in a **restricted sandbox environment** with access to specific control functions. All code snippets in this document are ready for direct execution without additional setup.

## System Architecture

### Execution Environment
- **Simulator**: MuJoCo physics engine with 3D visualization
- **Control Loop**: Real-time PID controller (mobile base) and position controller (arm)
- **Sandbox**: Restricted Python environment with limited builtins and exposed control functions
- **Threading**: Two threads - FastAPI server (with synchronous action execution) + MuJoCo simulator
- **Execution Model**: Actions execute **synchronously** - each HTTP request blocks until code execution completes

### Coordinate System
- **World Frame**: All positions and orientations are in world coordinates
- **Mobile Base**: Ground plane (x, y, theta)
  - x: forward/backward position in meters (world frame)
  - y: left/right position in meters (world frame)
  - theta: rotation angle in radians (counterclockwise positive)
- **Arm Joints**: 7-DOF Panda arm with joint angles in radians [j1, j2, j3, j4, j5, j6, j7]
- **End Effector**: Position [x, y, z] in meters (world frame), orientation [roll, pitch, yaw] in radians (world frame)
- **Objects**: All object positions and orientations are in world frame

### Control Strategy
- **Mobile Base**: PID velocity controller with position tracking
  - Gains: KP=[4.0, 4.0, 2.0], KI=[0.3, 0.3, 0.15], KD=[0.5, 0.5, 0.3]
  - Integral limits: I_LIMIT=[0.2, 0.2, 0.1]
  - Output: Velocity commands [vx, vy, omega]
- **Arm**: Position-controlled actuators (kp=1000, kv=100 in XML)
  - Direct joint position tracking with PD control

## Available Functions

### Mobile Base Control

#### `get_mobile_joint_position()`

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
pos = get_mobile_joint_position()
print(f"Base at x={pos[0]}, y={pos[1]}, theta={pos[2]}")

# Move relative to current position
current = get_mobile_joint_position()
new_pos = [current[0] + 1.0, current[1], current[2]]
set_mobile_target_joint(new_pos)
```

#### `set_mobile_target_joint(mobile_target_position, timeout=10.0, verbose=False)`

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

**Convergence Criteria:**
- Position error norm < 0.1 (with theta weighted at 50%)
- Velocity norm < 0.05 m/s or rad/s
- Must remain stable for 5 consecutive frames

**Returns:**
- `bool` or `None`: True if converged, False if timeout (when timeout > 0)

**Behavior:**
- Updates target position immediately (non-blocking simulator update)
- If timeout > 0, blocks HTTP request until convergence or timeout
- Adaptive sleep intervals based on distance to target (0.02s to 0.1s)

**Example:**
```python
# Move to position (1.0, 0.5, 0) and wait for arrival
set_mobile_target_joint([1.0, 0.5, 0])

# Move to position with verbose output
set_mobile_target_joint([2.0, -0.5, PI/2], verbose=True)

# Non-blocking update (returns immediately)
set_mobile_target_joint([0.0, 0.0, 0.0], timeout=0)
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

**Convergence Criteria:**
- Joint position error norm < 0.1 radians
- Joint velocity norm < 0.1 rad/s
- Must remain stable for 5 consecutive frames

**Returns:**
- `bool` or `None`: True if converged, False if timeout (when timeout > 0)

**Behavior:**
- Updates target position immediately (non-blocking simulator update)
- If timeout > 0, blocks HTTP request until convergence or timeout
- Adaptive sleep intervals based on distance to target (0.02s to 0.1s)

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

**Convergence Criteria:**
- Joint position error norm < 0.1 radians
- Joint velocity norm < 0.1 rad/s
- Must remain stable for 5 consecutive frames

**Limitations:**
- **Position only**: Cannot control orientation (roll, pitch, yaw)
- **IK solver**: Uses damped least squares IK with position-only targeting
- **World frame**: Target position is in world coordinates, not robot-relative

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

### Available Control Functions
The sandbox provides access to the following robot control functions:

**Mobile Base:**
- `get_mobile_joint_position()`: Get current base position [x, y, theta]
- `set_mobile_target_joint(position, timeout, verbose)`: Set base target

**Arm (Joint Space):**
- `get_arm_joint_position()`: Get current joint angles [j1~j7]
- `set_arm_target_joint(joints, timeout, verbose)`: Set joint targets

**End Effector (World Frame):**
- `get_ee_position()`: Get current EE pose (position, orientation) in world frame
- `set_ee_target_position(target_pos, timeout, verbose)`: Set EE target position in world frame (position only)

**Object Perception:**
- `get_object_positions()`: Get all object positions and orientations in world frame

### Builtin Functions
The sandbox provides access to the following Python builtins:
- `print(*args)`: Output text for debugging
- `range(start, stop, step)`: Generate numeric sequences
- `float(x)`: Convert to floating-point number
- `time`: Time module for delays and timing operations

**Example:**
```python
# Using available builtins
print("Moving to target position...")
for i in range(3):
    print(f"Step {i+1}")
    set_mobile_target_joint([float(i), 0, 0])
    time.sleep(1)
```

## Usage Examples

### Basic Mobile Base Movement

#### Move Forward 2 Meters
```python
set_mobile_target_joint([2.0, 0.0, 0.0])
```

#### Move in Square Pattern
```python
# Square with 1m sides
positions = [
    [1.0, 0.0, PI/2],    # Forward, turn left
    [1.0, 1.0, PI],      # Left, turn left
    [0.0, 1.0, -PI/2],   # Backward, turn left
    [0.0, 0.0, 0.0]      # Right, face forward
]

for pos in positions:
    set_mobile_target_joint(pos)
```

#### Rotate in Place
```python
# Rotate 180 degrees counterclockwise
set_mobile_target_joint([0.0, 0.0, PI])
```

### Basic Arm Movement

#### Move to Home Configuration
```python
home = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
set_arm_target_joint(home)
```

#### Reach Forward Configuration
```python
reach_forward = [0, 0.2, 0, -1.5, 0, 1.7, 0.785]
set_arm_target_joint(reach_forward)
```

### Combined Mobile Base and Arm Control

#### Navigate to Location and Reach
```python
# Move base to table
set_mobile_target_joint([1.5, 0.5, 0])

# Extend arm to reach object
reach_config = [0, 0.3, 0, -1.2, 0, 1.5, 0.785]
set_arm_target_joint(reach_config)
```

### End Effector Control Examples

#### Basic End Effector Movement
```python
# Move end effector to absolute world position
set_ee_target_position([1.5, -3.0, 1.2])

# Move to a different position
set_ee_target_position([2.0, -2.5, 1.0])

# Move with verbose output
set_ee_target_position([1.8, -3.2, 1.3], verbose=True)
```

#### Pick and Place Pattern
```python
# Get object position
objects = get_object_positions()
if 'object_apple_0' in objects:
    apple_pos = objects['object_apple_0']['pos']
    print(f"Apple at: {apple_pos}")
    
    # Move above the apple
    approach_pos = [apple_pos[0], apple_pos[1], apple_pos[2] + 0.2]
    set_ee_target_position(approach_pos)
    
    # Lower to grasp height
    grasp_pos = [apple_pos[0], apple_pos[1], apple_pos[2] + 0.05]
    set_ee_target_position(grasp_pos)
    
    # Lift object
    lift_pos = [apple_pos[0], apple_pos[1], apple_pos[2] + 0.3]
    set_ee_target_position(lift_pos)
    
    # Move to placement location
    place_pos = [2.5, -2.8, 1.0]
    set_ee_target_position(place_pos)
```

#### Scan Multiple Objects
```python
# Get all objects
objects = get_object_positions()

# Visit each object
for name, obj in objects.items():
    print(f"Moving to {name}")
    # Move 15cm above each object
    target = [obj['pos'][0], obj['pos'][1], obj['pos'][2] + 0.15]
    success = set_ee_target_position(target, verbose=True)
    if success:
        print(f"Reached {name}")
    else:
        print(f"Failed to reach {name}")
```

#### Sequential Multi-Point Navigation
```python
# Navigate through waypoints
waypoints = [
    [1.0, 0.0, 0],
    [2.0, 0.5, PI/4],
    [1.5, 1.0, PI/2]
]

for waypoint in waypoints:
    print(f"Moving to {waypoint}")
    set_mobile_target_joint(waypoint, verbose=True)
    time.sleep(0.5)  # Brief pause between waypoints
```

### Non-Blocking Control Pattern

#### Simultaneous Base and Arm Motion
```python
# Issue commands without waiting
set_mobile_target_joint([2.0, 1.0, PI/4], timeout=0)
set_arm_target_joint([0, 0.5, 0, -1.0, 0, 1.5, 0.785], timeout=0)

# Wait for both to settle (manual wait)
time.sleep(5)
```

### Advanced: Combined Motion Patterns

#### Mobile Base Circle with Arm Movement
```python
import time

radius = 1.0
num_points = 8

for i in range(num_points + 1):
    angle = 2 * PI * i / num_points
    x = radius * (1 - float(i) / num_points)
    y = radius * (1 - float(i) / num_points) * angle / (2 * PI)
    theta = angle

    set_mobile_target_joint([x, y, theta], timeout=5, verbose=True)

    # Adjust end effector height at each point
    if i > 0:
        pos, ori = get_ee_position()
        new_pos = [pos[0], pos[1], pos[2] + 0.02]
        set_ee_target_position(new_pos)
```

#### Coordinate Mobile Base and EE for Object Collection
```python
# Move base to different locations and collect objects
base_positions = [
    [1.8, -3.0, 0],
    [2.5, -2.8, 0],
    [3.2, -3.2, 0]
]

for base_pos in base_positions:
    print(f"Moving base to {base_pos}")
    set_mobile_target_joint(base_pos)
    
    # Find nearby objects
    objects = get_object_positions()
    for name, obj in objects.items():
        # Check if object is within reach
        dx = obj['pos'][0] - base_pos[0]
        dy = obj['pos'][1] - base_pos[1]
        dist = (dx**2 + dy**2) ** 0.5
        
        if dist < 0.8:  # Within 80cm reach
            print(f"  Reaching for {name}")
            target = [obj['pos'][0], obj['pos'][1], obj['pos'][2] + 0.1]
            set_ee_target_position(target, timeout=5)
```

## Parameters and Safety Information

### Action Submission Format
When sending code via REST API:
```json
{
  "action": {
    "type": "run_code",
    "payload": {
      "code": "set_mobile_target_joint([1.0, 0.5, 0])"
    }
  }
}
```

### Safety Constraints

**Mobile Base Limits:**
- Velocity commands computed by PID controller
- Physical workspace constraints enforced by MuJoCo collision detection
- Timeout prevents infinite waiting (max 10s default)

**Arm Limits:**
- Joint limits: [-2.9, 2.9] rad for all 7 joints
- Position-controlled actuators prevent excessive velocities
- Convergence thresholds prevent premature motion termination

**End Effector Control:**
- **IK solver**: Damped least squares IK for position-only targeting
- **Position only**: Can set absolute position but not orientation
- **No orientation control**: Roll, pitch, yaw cannot be commanded
- **World frame**: All positions are in world coordinates

**Execution Environment:**
- **Synchronous execution**: HTTP request blocks until code completes
- **Single action at a time**: Concurrent requests queue at FastAPI level
- **Sandboxed execution**: No file system access or dangerous operations
- **No network access**: Subprocess spawning not allowed

### Convergence Tuning

**Position Thresholds:**
- Mobile base: 0.1m position error, 0.05 m/s velocity
- Arm (joint space): 0.1 rad position error, 0.1 rad/s velocity
- End effector (IK): Same as arm joint convergence

**Stability Requirements:**
- Must maintain threshold for 5 consecutive frames (~50ms at typical sim rate)
- Prevents false convergence during oscillations

**Timeout Recommendations:**
- Short moves (<1m or <60°): 5-10 seconds
- Long moves (>2m or >120°): 10-20 seconds
- End effector positioning: 5-10 seconds
- Complex trajectories: Consider non-blocking + manual timing

### Debugging Tips

**Enable Verbose Output:**
```python
set_mobile_target_joint([1.0, 0.0, 0], verbose=True)
# Output: "Converged after 3.42s (342 iterations)"
```

**Check Convergence Status:**
```python
success = set_mobile_target_joint([2.0, 1.0, PI/2], timeout=5)
if not success:
    print("Warning: Motion did not converge within timeout")

# Check end effector positioning success
success = set_ee_target_position([1.5, -3.0, 1.2], timeout=5)
if not success:
    print("End effector positioning failed or timed out")
```

**Progressive Movement:**
```python
# Break long moves into shorter segments
waypoints = [[0.5, 0, 0], [1.0, 0, 0], [1.5, 0, 0]]
for wp in waypoints:
    set_mobile_target_joint(wp, timeout=3, verbose=True)
```

**Monitor End Effector Position:**
```python
# Check pose before and after movement
print("Before:", get_ee_position())
set_ee_target_position([1.5, -3.0, 1.2], verbose=True)
print("After:", get_ee_position())
```

### Common Patterns

**Wait Between Actions:**
```python
set_mobile_target_joint([1.0, 0, 0])
time.sleep(1)  # Pause before next command
set_arm_target_joint([0, 0, 0, -PI/2, 0, PI/2, PI/4])
```

**Loop with Range:**
```python
for i in range(5):
    x = float(i) * 0.5
    set_mobile_target_joint([x, 0, 0], verbose=True)
```

**Conditional Execution:**
```python
# Example: Try movement with fallback
success = set_mobile_target_joint([3.0, 0, 0], timeout=8)
if not success:
    print("Long move failed, trying shorter distance")
    set_mobile_target_joint([2.0, 0, 0], timeout=5)
```

**Return Values via RESULT Dictionary:**
```python
# Store results for API response
pos, ori = get_ee_position()
RESULT['end_effector_position'] = pos
RESULT['end_effector_orientation'] = ori
RESULT['status'] = 'completed'
```

## Limitations and Future Enhancements

### Current Limitations
- **No orientation control**: Cannot command end effector orientation (roll, pitch, yaw)
- **No gripper control**: Gripper functions not yet exposed to sandbox
- **No force/torque sensing**: Sensor data not currently accessible
- **Synchronous execution only**: Cannot run multiple actions concurrently
- **Read-only object perception**: Can read object positions but cannot manipulate them directly

### Recent Enhancements
- **IK solver**: Damped least squares IK for position-only end effector control
- **World frame control**: All positions in consistent world coordinate frame
- **Object perception**: Can query positions and orientations of all scene objects

### Potential Future Enhancements
- Full 6-DOF end effector control (position + orientation)
- Gripper open/close commands
- Force/torque feedback for contact detection
- Asynchronous action execution
- Object grasping and manipulation primitives
