# Mobile Manipulator Robot Control API Documentation

## Table of Contents
1. [Introduction](#introduction)
2. [System Architecture](#system-architecture)
3. [Available Functions](#available-functions)
   - [Mobile Base Control](#mobile-base-control)
   - [Arm Control](#arm-control)
4. [Available Constants and Utilities](#available-constants-and-utilities)
5. [Usage Examples](#usage-examples)
6. [Parameters and Safety Information](#parameters-and-safety-information)

## Introduction

This documentation provides detailed instructions for controlling a Panda-Omron mobile manipulator in a MuJoCo simulation environment. The robot operates in a kitchen scene and can be controlled through sandboxed Python code execution via a REST API.

User-submitted code executes in a **restricted sandbox environment** with access to specific control functions. All code snippets in this document are ready for direct execution without additional setup.

## System Architecture

### Execution Environment
- **Simulator**: MuJoCo physics engine with 3D visualization
- **Control Loop**: Real-time PD controller running at simulation timestep frequency
- **Sandbox**: Restricted Python environment with limited builtins and exposed control functions
- **Threading**: Non-blocking control updates + blocking convergence waiting

### Coordinate System
- **Mobile Base**: Ground plane (x, y, theta)
  - x: forward/backward position in meters
  - y: left/right position in meters
  - theta: rotation angle in radians (counterclockwise positive)
- **Arm Joints**: 7-DOF Panda arm with joint angles in radians [j1, j2, j3, j4, j5, j6, j7]

### Control Strategy
- **Mobile Base**: PD velocity controller with position tracking
  - Gains: KP=[2.0, 2.0, 1.5], KD=[0.5, 0.5, 0.3]
  - Output: Velocity commands [vx, vy, omega]
- **Arm**: Position-controlled actuators (kp=1000, kv=100)
  - Direct joint position tracking

## Available Functions

### Mobile Base Control

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

**Convergence Criteria:**
- Position error norm < 0.1 (with theta weighted at 50%)
- Velocity norm < 0.05 m/s or rad/s
- Must remain stable for 5 consecutive frames

**Returns:**
- `bool` or `None`: True if converged, False if timeout (when timeout > 0)

**Behavior:**
- Updates target position immediately (non-blocking simulator update)
- If timeout > 0, blocks action processor thread until convergence or timeout
- Adaptive sleep intervals based on distance to target (0.02s to 0.1s)

**Example:**
```python
# Move to position (1.0, 0.5, 0) and wait for arrival
set_mobile_target_position([1.0, 0.5, 0])

# Move to position with verbose output
set_mobile_target_position([2.0, -0.5, PI/2], verbose=True)

# Non-blocking update (returns immediately)
set_mobile_target_position([0.0, 0.0, 0.0], timeout=0)
```

### Arm Control

#### `set_arm_target_position(arm_target_position, timeout=10.0, verbose=False)`

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
- If timeout > 0, blocks action processor thread until convergence or timeout
- Adaptive sleep intervals based on distance to target (0.02s to 0.1s)

**Example:**
```python
# Move arm to home configuration and wait
home_position = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
set_arm_target_position(home_position)

# Move with verbose output
set_arm_target_position([0, 0, 0, -PI/2, 0, PI/2, PI/4], verbose=True)

# Non-blocking update
set_arm_target_position([0.1, -0.5, 0.2, -1.8, 0.3, 1.2, 0.9], timeout=0)
```

## Available Constants and Utilities

### Constants
- `PI`: Mathematical constant π (numpy.pi ≈ 3.14159265359)

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
    set_mobile_target_position([float(i), 0, 0])
    time.sleep(1)
```

## Usage Examples

### Basic Mobile Base Movement

#### Move Forward 2 Meters
```python
set_mobile_target_position([2.0, 0.0, 0.0])
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
    set_mobile_target_position(pos)
```

#### Rotate in Place
```python
# Rotate 180 degrees counterclockwise
set_mobile_target_position([0.0, 0.0, PI])
```

### Basic Arm Movement

#### Move to Home Configuration
```python
home = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
set_arm_target_position(home)
```

#### Reach Forward Configuration
```python
reach_forward = [0, 0.2, 0, -1.5, 0, 1.7, 0.785]
set_arm_target_position(reach_forward)
```

### Combined Mobile Base and Arm Control

#### Navigate to Location and Reach
```python
# Move base to table
set_mobile_target_position([1.5, 0.5, 0])

# Extend arm to reach object
reach_config = [0, 0.3, 0, -1.2, 0, 1.5, 0.785]
set_arm_target_position(reach_config)
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
    set_mobile_target_position(waypoint, verbose=True)
    time.sleep(0.5)  # Brief pause between waypoints
```

### Non-Blocking Control Pattern

#### Simultaneous Base and Arm Motion
```python
# Issue commands without waiting
set_mobile_target_position([2.0, 1.0, PI/4], timeout=0)
set_arm_target_position([0, 0.5, 0, -1.0, 0, 1.5, 0.785], timeout=0)

# Wait for both to settle (manual wait)
time.sleep(5)
```

### Advanced: Circle Trajectory

#### Mobile Base Circle
```python
import time

radius = 1.0
num_points = 12

for i in range(num_points + 1):
    angle = 2 * PI * i / num_points
    x = radius * (1 - float(i) / num_points)  # Using float() builtin
    y = radius * (1 - float(i) / num_points) * angle / (2 * PI)
    theta = angle

    set_mobile_target_position([x, y, theta], timeout=5, verbose=True)
```

## Parameters and Safety Information

### Action Submission Format
When sending code via REST API:
```json
{
  "action": {
    "type": "run_code",
    "payload": {
      "code": "set_mobile_target_position([1.0, 0.5, 0])"
    }
  }
}
```

### Safety Constraints

**Mobile Base Limits:**
- Velocity commands computed by PD controller
- Physical workspace constraints enforced by MuJoCo collision detection
- Timeout prevents infinite waiting (max 10s default)

**Arm Limits:**
- Joint limits defined in robot model XML
- Position-controlled actuators prevent excessive velocities
- Convergence thresholds prevent premature motion termination

**Execution Environment:**
- Single-threaded action processing (one action executes at a time)
- Sandboxed execution prevents file system access and dangerous operations
- No network access or subprocess spawning allowed

### Convergence Tuning

**Position Thresholds:**
- Mobile base: 0.1m position error, 0.05 m/s velocity
- Arm: 0.1 rad position error, 0.1 rad/s velocity

**Stability Requirements:**
- Must maintain threshold for 5 consecutive frames (~50ms at typical sim rate)
- Prevents false convergence during oscillations

**Timeout Recommendations:**
- Short moves (<1m or <60°): 5-10 seconds
- Long moves (>2m or >120°): 10-20 seconds
- Complex trajectories: Consider non-blocking + manual timing

### Debugging Tips

**Enable Verbose Output:**
```python
set_mobile_target_position([1.0, 0.0, 0], verbose=True)
# Output: "Converged after 3.42s (342 iterations)"
```

**Check Convergence Status:**
```python
success = set_mobile_target_position([2.0, 1.0, PI/2], timeout=5)
if not success:
    print("Warning: Motion did not converge within timeout")
```

**Progressive Movement:**
```python
# Break long moves into shorter segments
waypoints = [[0.5, 0, 0], [1.0, 0, 0], [1.5, 0, 0]]
for wp in waypoints:
    set_mobile_target_position(wp, timeout=3, verbose=True)
```

### Common Patterns

**Wait Between Actions:**
```python
set_mobile_target_position([1.0, 0, 0])
time.sleep(1)  # Pause before next command
set_arm_target_position([0, 0, 0, -PI/2, 0, PI/2, PI/4])
```

**Loop with Range:**
```python
for i in range(5):
    x = float(i) * 0.5
    set_mobile_target_position([x, 0, 0], verbose=True)
```

**Conditional Execution:**
```python
# Example: Try movement with fallback
success = set_mobile_target_position([3.0, 0, 0], timeout=8)
if not success:
    print("Long move failed, trying shorter distance")
    set_mobile_target_position([2.0, 0, 0], timeout=5)
```
