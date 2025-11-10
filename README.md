# MuJoCo Robot Simulator API

A REST API-based control system for Panda-Omron mobile manipulator simulation using MuJoCo physics engine. Execute Python code remotely to control a simulated robot in a kitchen environment.

## Features

- **Real-time Physics Simulation**: High-fidelity MuJoCo simulation with 3D visualization
- **REST API Control**: Send Python code via HTTP to control the robot
- **Sandboxed Execution**: Safe code execution environment with limited access
- **Mobile Base & Arm Control**: Holonomic drive system + 7-DOF Panda arm
- **PD Controller**: Automatic position tracking with velocity-based convergence
- **Asynchronous Processing**: Non-blocking action queue with optional blocking wait
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

### Running the Simulator

```bash
cd robot
python main.py # or mjpython main.py
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
      "code": "set_mobile_target_position([0, 0, PI])"
    }
  }
}
```

### Example Code

#### Mobile Base Control

Move robot to position (x=0, y=0, theta=π):
```python
set_mobile_target_position([0, 0, PI])
```

Move robot with verbose output and custom timeout:
```python
set_mobile_target_position([2.0, 1.0, PI/2], timeout=5.0, verbose=True)
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
    set_mobile_target_position(pos)
```

#### Arm Control

Move arm to home configuration:
```python
home = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
set_arm_target_position(home)
```

Move arm with verbose monitoring:
```python
reach_config = [0, 0.2, 0, -1.5, 0, 1.7, 0.785]
set_arm_target_position(reach_config, verbose=True)
```

#### Combined Control

Navigate and reach:
```python
# Move base to location
set_mobile_target_position([1.5, 0.5, 0])

# Extend arm to reach object
set_arm_target_position([0, 0.3, 0, -1.2, 0, 1.5, 0.785])
```

### Available Functions in Sandbox

#### Mobile Base Control
- `set_mobile_target_position(mobile_target_position, timeout=10.0, verbose=False)`
  - `mobile_target_position`: List [x, y, theta] in meters and radians
  - `timeout`: Maximum wait time in seconds (default: 10.0, set 0 for non-blocking)
  - `verbose`: Print convergence progress (default: False)
  - **Convergence**: Position error < 0.1m, velocity < 0.05 m/s, stable for 5 frames
  - **Returns**: True if converged, False if timeout

#### Arm Control
- `set_arm_target_position(arm_target_position, timeout=10.0, verbose=False)`
  - `arm_target_position`: List [j1, j2, j3, j4, j5, j6, j7] in radians
  - `timeout`: Maximum wait time in seconds (default: 10.0, set 0 for non-blocking)
  - `verbose`: Print convergence progress (default: False)
  - **Convergence**: Joint error < 0.1 rad, velocity < 0.1 rad/s, stable for 5 frames
  - **Returns**: True if converged, False if timeout

#### Utilities
- `print()`: Print debug messages
- `range()`, `float()`, `time`: Standard Python builtins
- `PI`: Constant for π (3.14159...)

For complete API documentation, see [robot/code_knowledge.md](robot/code_knowledge.md)

## Project Structure

```
with-robot-4th-lab/
├── robot/
│   ├── main.py              # FastAPI server and threading orchestration
│   ├── simulator.py         # MuJoCo simulator with PD controller
│   ├── code_repository.py   # Sandboxed code execution layer
│   ├── code_knowledge.md    # Mobile manipulator API documentation (LLM-ready)
│   ├── ellmer_knowledge.md  # Kinova robot arm reference examples
│   └── client.ipynb         # Jupyter notebook for testing API
├── model/
│   └── robocasa/            # From RoboCasa project
│       ├── panda_omron.xml  # Robot model (default)
│       └── assets/          # Meshes, textures, and scene objects
├── requirements.txt         # Python dependencies
├── CLAUDE.md               # Development documentation
└── README.md               # This file
```

## Architecture

The system uses a three-layer architecture:

1. **simulator.py**: Core MuJoCo physics simulation with PD controller
2. **code_repository.py**: Sandboxed Python execution environment
3. **main.py**: FastAPI server with three concurrent threads:
   - Main thread: HTTP request handling
   - Simulator thread: Physics simulation and 3D rendering
   - Action processor thread: Asynchronous code execution

## Configuration

### Robot Control Parameters

Edit `robot/simulator.py` to adjust:

```python
# PD controller gains
KP = np.array([2.0, 2.0, 1.5])  # Position gains [x, y, theta]
KD = np.array([0.5, 0.5, 0.3])  # Derivative gains [x, y, theta]

# Camera view settings
CAM_LOOKAT = [2.15, -0.8, 0.8]
CAM_DISTANCE = 5.0
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

The robot model XML and scene assets are from the [RoboCasa](https://github.com/robocasa/robocasa) project. Change the MuJoCo model in `robot/simulator.py`:

```python
# Line 40
xml_path = "../model/robocasa/panda_omron.xml"  # Default from RoboCasa
```

## Technical Details

- **Physics Engine**: MuJoCo 3.3.7
- **Control System**:
  - PD controller for mobile base velocity tracking
  - Position control for 7-DOF Panda arm
- **Convergence Logic**:
  - Position + velocity stability checks
  - Adaptive sleep intervals (0.02s-0.1s based on error)
  - Configurable timeout with verbose monitoring
- **Actuators**:
  - Position control (kp=1000, kv=100) for Panda arm
  - Velocity control (kv=1000/1500) for mobile base
- **Sensors**: Force/torque sensors on gripper end-effector
- **Threading**: Daemon threads for clean shutdown
- **Safety**: Sandboxed code execution with restricted builtins

### Convergence Criteria

**Mobile Base:**
- Position error norm < 0.1m (theta weighted at 50%)
- Velocity norm < 0.05 m/s or rad/s
- Stability: 5 consecutive frames within threshold

**Arm:**
- Joint position error norm < 0.1 radians
- Joint velocity norm < 0.1 rad/s
- Stability: 5 consecutive frames within threshold

## License

See [LICENSE](LICENSE) file for details.

## Contributing

This is a research/educational project. For questions or contributions, please open an issue.

## Acknowledgments

- **MuJoCo Physics Engine**: Physics simulation by DeepMind
- **Panda Robot Model**: 7-DOF manipulator by Franka Emika
- **RoboCasa Project**: Robot model XML (`panda_omron.xml`) and kitchen scene assets (meshes, textures, objects) from [RoboCasa](https://github.com/robocasa/robocasa)
  - Provides realistic kitchen environment for mobile manipulation research
