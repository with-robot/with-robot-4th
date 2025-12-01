# MuJoCo Robot Simulator API

A REST API-based control system for Panda-Omron mobile manipulator simulation using MuJoCo physics engine. Execute Python code remotely to control a simulated robot in a kitchen environment.

## Features

- **Real-time Physics Simulation**: High-fidelity MuJoCo simulation with 3D visualization
- **REST API Control**: Send Python code via HTTP to control the robot
- **Sandboxed Execution**: Safe code execution environment with limited access
- **Mobile Base & Arm Control**: Holonomic drive system + 7-DOF Panda arm
- **Path Planning**: A* algorithm for collision-free navigation with obstacle avoidance
- **Grid Map**: Binary occupancy grid map for environment perception
- **Gripper Control**: Width-based gripper control for grasping objects
- **End Effector Control**: IK-based position control in world frame
- **Object Perception**: Query positions and orientations of all scene objects
- **PID Controller**: Mobile base with integral term for steady-state error elimination
- **Synchronous Processing**: Blocking HTTP requests ensure action completion
- **Adaptive Convergence**: Smart waiting with position + velocity stability checks

## Quick Start

### Prerequisites

- Python 3.8+
- MuJoCo physics engine support

### Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd with-robot-4th
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

Dependencies:
- FastAPI 0.121.1
- MuJoCo 3.3.7
- uvicorn 0.38.0
- scipy >= 1.10.0

### Running the Simulator

```bash
cd robot
python main.py
```

The server will start on `http://0.0.0.0:8800` with:
- Interactive API documentation at `http://localhost:8800/docs`
- 3D MuJoCo viewer window for real-time visualization

## Usage

### API Endpoints

#### Health Check
```bash
GET http://localhost:8800/
```

Response:
```json
{
  "name": "MuJoCo Robot Simulator",
  "version": "0.0.1",
  "status": "running"
}
```

#### Send Action
```bash
POST http://localhost:8800/send_action
Content-Type: application/json

{
  "action": {
    "type": "run_code",
    "payload": {
      "code": "set_mobile_target_joint([0, 0, PI])"
    }
  }
}
```

### Example Code

#### Mobile Base Control

Move robot to position (x=0, y=0, theta=π):
```python
set_mobile_target_joint([0, 0, PI])
```

Move robot with verbose output and custom timeout:
```python
set_mobile_target_joint([2.0, 1.0, PI/2], timeout=5.0, verbose=True)
```

Move robot in square pattern:
```python
positions = [
    [1.0, 0.0, PI/2],    # Forward, turn left
    [1.0, 1.0, PI],      # Left, turn left
    [0.0, 1.0, -PI/2],   # Backward, turn left
    [0.0, 0.0, 0.0]      # Right, face forward
]

for pos in positions:
    set_mobile_target_joint(pos)
```

#### Path Planning Example

Plan and execute collision-free path:
```python
# Plan path to target location
path = plan_mobile_path([2.0, -3.0])
if path is not None:
    print(f"Found path with {len(path)} waypoints")
    # Execute path by visiting each waypoint
    for waypoint in path:
        set_mobile_target_joint(waypoint, verbose=True)
else:
    print("No path found to target")
```

Get environment grid map:
```python
# Get binary occupancy grid (0=free, 1=occupied)
grid = get_grid_map()
print(f"Grid size: {len(grid)} x {len(grid[0])}")

# Check if a cell is free
if grid[50][50] == 0:
    print("Cell is navigable")
```

#### Arm Control

Move arm to home configuration:
```python
home = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
set_arm_target_joint(home)
```

Move arm with verbose monitoring:
```python
reach_config = [0, 0.2, 0, -1.5, 0, 1.7, 0.785]
set_arm_target_joint(reach_config, verbose=True)
```

#### End Effector Control (World Frame)

Move end effector to absolute world position:
```python
set_ee_target_position([1.5, -3.0, 1.2])
```

Get current end effector pose:
```python
# Returns tuple: (position, orientation) in world frame
pos, ori = get_ee_position()
print(pos)  # [x, y, z] in meters
print(ori)  # [roll, pitch, yaw] in radians
```

Pick and place pattern with gripper:
```python
# Get object position
objects = get_object_positions()
if 'object_apple_0' in objects:
    apple_pos = objects['object_apple_0']['pos']
    
    # Open gripper
    set_target_gripper_width(0.08)
    
    # Move above the apple
    approach_pos = [apple_pos[0], apple_pos[1], apple_pos[2] + 0.2]
    set_ee_target_position(approach_pos)
    
    # Lower to grasp height
    grasp_pos = [apple_pos[0], apple_pos[1], apple_pos[2] + 0.05]
    set_ee_target_position(grasp_pos)
    
    # Close gripper to grasp
    set_target_gripper_width(0.02)
    
    # Lift object
    lift_pos = [apple_pos[0], apple_pos[1], apple_pos[2] + 0.3]
    set_ee_target_position(lift_pos)
    
    # Move to placement location
    place_pos = [2.5, -2.8, 1.0]
    set_ee_target_position(place_pos)
    
    # Open gripper to release
    set_target_gripper_width(0.08)
```

#### Object Perception

Get all object positions:
```python
# Returns dictionary mapping object names to their properties
objects = get_object_positions()

for name, obj in objects.items():
    print(f"{name}: pos={obj['pos']}, ori={obj['ori']}")

# Access specific object
if 'object_banana_1' in objects:
    banana = objects['object_banana_1']
    target = [banana['pos'][0], banana['pos'][1], banana['pos'][2] + 0.1]
    set_ee_target_position(target)
```

#### Combined Control

Navigate and reach:
```python
# Move base to location
set_mobile_target_joint([1.5, 0.5, 0])

# Extend arm to reach object
set_arm_target_joint([0, 0.3, 0, -1.2, 0, 1.5, 0.785])
```

Get current robot state:
```python
# Get mobile base position [x, y, theta]
base_pos = get_mobile_joint_position()

# Get arm joint angles [j1~j7]
arm_pos = get_arm_joint_position()

# Get end effector pose
pos, ori = get_ee_position()
```

### Available Functions in Sandbox

#### Mobile Base Control
- `get_mobile_joint_position()`
  - **Returns**: List [x, y, theta] - current base position in meters and radians

- `set_mobile_target_joint(mobile_target_position, timeout=10.0, verbose=False)`
  - `mobile_target_position`: List [x, y, theta] in meters and radians
  - `timeout`: Maximum wait time in seconds (default: 10.0, set 0 for non-blocking)
  - `verbose`: Print convergence progress (default: False)
  - **Convergence**: Position error < 0.1m, velocity < 0.05 m/s, stable for 5 frames
  - **Returns**: True if converged, False if timeout

- `plan_mobile_path(target_joint, grid_size=0.1)`
  - `target_joint`: Target position [x, y] in world coordinates
  - `grid_size`: Grid cell size in meters (default: 0.1)
  - **Returns**: List of waypoints [(x, y, theta), ...] or None if unreachable
  - **Uses**: A* pathfinding with obstacle inflation and path simplification

- `get_grid_map()`
  - **Returns**: 2D binary occupancy grid (0=free, 1=occupied)
  - Grid cell size: 0.1m, includes all static obstacles

#### Arm Control (Joint Space)
- `get_arm_joint_position()`
  - **Returns**: List [j1~j7] - current joint angles in radians

- `set_arm_target_joint(arm_target_position, timeout=10.0, verbose=False)`
  - `arm_target_position`: List [j1, j2, j3, j4, j5, j6, j7] in radians
  - `timeout`: Maximum wait time in seconds (default: 10.0, set 0 for non-blocking)
  - `verbose`: Print convergence progress (default: False)
  - **Convergence**: Joint error < 0.1 rad, velocity < 0.1 rad/s, stable for 5 frames
  - **Returns**: True if converged, False if timeout

#### End Effector Control (World Frame)
- `get_ee_position()`
  - **Returns**: Tuple (position, orientation) where position=[x,y,z] in meters (world frame), orientation=[roll,pitch,yaw] in radians (world frame)

- `set_ee_target_position(target_pos, timeout=10.0, verbose=False)`
  - `target_pos`: Target position [x, y, z] in meters (world frame)
  - `timeout`: Maximum wait time in seconds (default: 10.0, set 0 for non-blocking)
  - `verbose`: Print convergence progress (default: False)
  - **Note**: Position only - no orientation control, uses IK solver
  - **Returns**: True if IK succeeded and converged, False if IK failed or timeout

#### Gripper Control
- `get_gripper_width()`
  - **Returns**: Float - current gripper width in meters

- `set_target_gripper_width(target_width, timeout=10.0, verbose=False)`
  - `target_width`: Target gripper width in meters (0.0=closed, 0.08=fully open)
  - `timeout`: Maximum wait time in seconds (default: 10.0, set 0 for non-blocking)
  - `verbose`: Print convergence progress (default: False)
  - **Convergence**: Width error < 0.02m, velocity < 0.02 m/s, stable for 5 frames
  - **Note**: Includes 1 second stabilization delay after convergence
  - **Returns**: True if converged, False if timeout

#### Object Perception
- `get_object_positions()`
  - **Returns**: Dictionary mapping object names to their properties
    - Key: object name (str)
    - Value: dict with 'id' (int), 'pos' (list[float]), 'ori' (list[float])
  - All positions and orientations are in world frame

#### Utilities
- `print()`: Print debug messages
- `range()`, `float()`, `time`: Standard Python builtins
- `PI`: Constant for π (3.14159...)
- `RESULT`: Dictionary for storing return values from user code

For complete API documentation, see [robot/code_knowledge.md](robot/code_knowledge.md)

## Project Structure

```
with-robot-4th-lab/
├── robot/
│   ├── main.py               # FastAPI server and threading orchestration
│   ├── simulator.py          # MuJoCo simulator with PID controller
│   ├── code_repository.py    # Sandboxed code execution layer
│   ├── code_knowledge.md     # Mobile manipulator API documentation (LLM-ready)
│   ├── ellmer_knowledge.md   # Kinova robot arm reference examples
│   └── client.ipynb          # Jupyter notebook for testing API
├── model/
│   └── robocasa/             # From RoboCasa project
│       ├── site.xml          # Main scene file (entry point)
│       ├── panda_omron.xml   # Robot model definition
│       ├── fixtures.xml      # Kitchen fixtures (fridge, oven, etc.)
│       ├── geom_objects.xml  # Temporary geom objects (cubes, etc.)
│       ├── objects/          # Object definitions (fruits, utensils, etc.)
│       ├── assets/           # Meshes and textures
│       └── backup/           # Backup files
├── requirements.txt          # Python dependencies
└── README.md                 # This file
```

## Architecture

The system uses a two-thread architecture:

1. **Main Thread**: FastAPI uvicorn server handling HTTP requests with synchronous code execution
2. **Simulator Thread**: MuJoCo physics simulation with 3D rendering

**Execution Model**: Actions execute synchronously - each HTTP request blocks until code execution completes.

**Component Layers**:
1. **simulator.py**: Core MuJoCo physics simulation with PID controller (mobile base), position controller (arm), and IK solver (end effector)
2. **code_repository.py**: Sandboxed Python execution environment
3. **main.py**: FastAPI server orchestrating simulator and action execution

## Configuration

### Robot Control Parameters

Edit `robot/simulator.py` to adjust:

```python
# PID controller gains for mobile base
MOBILE_KP = np.array([4.0, 4.0, 2.0])       # Position gains [x, y, theta]
MOBILE_KI = np.array([0.3, 0.3, 0.15])      # Integral gains
MOBILE_I_LIMIT = np.array([0.2, 0.2, 0.1])  # Integral limits
MOBILE_KD = np.array([0.5, 0.5, 0.3])       # Derivative gains

# Camera view settings
CAM_LOOKAT = [-0.8, -0.8, 0.8]
CAM_DISTANCE = 7.5
CAM_AZIMUTH = 135
CAM_ELEVATION = -25
```

### Server Settings

Edit `robot/main.py` to change:

```python
HOST = "0.0.0.0"  # Listen on all interfaces
PORT = 8800       # API server port
```

### Robot Model

The robot model XML and scene assets are from the [RoboCasa](https://github.com/robocasa/robocasa) project. The main scene file is loaded in `robot/simulator.py`:

```python
# Line 80
xml_path = "../model/robocasa/site.xml"  # Main scene entry point
```

The scene is organized as:
- `site.xml`: Main scene file that includes robot, fixtures, and objects
- `panda_omron.xml`: Robot model definition (Panda arm + Omron mobile base)
- `fixtures.xml`: Kitchen environment (fridge, oven, sink, counters, etc.)
- `objects/`: Individual object definitions (fruits, utensils, bowls, etc.)

## Technical Details

- **Physics Engine**: MuJoCo 3.3.7
- **Control System**:
  - PID controller for mobile base velocity tracking
  - Position control for 7-DOF Panda arm
  - IK solver (damped least squares) for end effector position control
- **Coordinate System**: All positions and orientations in world frame
- **Convergence Logic**:
  - Position + velocity stability checks
  - Adaptive sleep intervals (0.02s-0.1s based on error)
  - Configurable timeout with verbose monitoring
- **Actuators**:
  - Position control (kp=1000, kv=100) for Panda arm
  - Velocity control (kv=1000/1500) for mobile base
- **Sensors**: Force/torque sensors on gripper end-effector
- **Threading**: Two daemon threads for clean shutdown
- **Safety**: Sandboxed code execution with restricted builtins

### Convergence Criteria

**Mobile Base:**
- Position error norm < 0.1m (theta weighted at 50%)
- Velocity norm < 0.05 m/s or rad/s
- Stability: 5 consecutive frames within threshold

**Arm (Joint Space):**
- Joint position error norm < 0.1 radians
- Joint velocity norm < 0.1 rad/s
- Stability: 5 consecutive frames within threshold

**End Effector (IK-based Position Control):**
- Joint position error norm < 0.1 radians
- Joint velocity norm < 0.1 rad/s
- Stability: 5 consecutive frames within threshold
- IK solver: Damped least squares for position-only targeting
- No orientation control

## Current Limitations

- **No orientation control**: Cannot command end effector orientation (roll, pitch, yaw)
- **Synchronous execution only**: Cannot run multiple actions concurrently
- **Read-only object perception**: Can read object positions but cannot manipulate them directly

## Recent Enhancements

- **Gripper control**: Width-based gripper control for grasping and releasing objects
- **IK solver**: Damped least squares IK for position-only end effector control
- **World frame control**: All positions in consistent world coordinate frame
- **Object perception**: Can query positions and orientations of all scene objects

See [CLAUDE.md](CLAUDE.md) for development guidance on adding features.

## License

See [LICENSE](LICENSE) file for details.

## Contributing

This is a research/educational project. For questions or contributions, please open an issue.

## Acknowledgments

- **MuJoCo Physics Engine**: Physics simulation by DeepMind
- **Panda Robot Model**: 7-DOF manipulator by Franka Emika
- **RoboCasa Project**: Robot model XML (`panda_omron.xml`) and kitchen scene assets (meshes, textures, objects) from [RoboCasa](https://github.com/robocasa/robocasa)
  - Provides realistic kitchen environment for mobile manipulation research
