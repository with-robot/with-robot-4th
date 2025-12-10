# MuJoCo Robot Simulator API

A REST API-based control system for Panda-Omron mobile manipulator simulation using MuJoCo physics engine. Execute Python code remotely to control a simulated robot in a kitchen environment.

## Features

- **Real-time Physics Simulation**: High-fidelity MuJoCo simulation with 3D visualization
- **REST API Control**: Send Python code via HTTP to control the robot
- **Sandboxed Execution**: Safe code execution environment with limited access
- **Mobile Base Control**: Holonomic drive system with independent x, y, theta control
- **PD Controller**: Automatic position tracking with configurable gains
- **Asynchronous Processing**: Non-blocking action queue for smooth operation
- **Korean Voice Commands**: ElevenLabs STT/TTS for natural language robot control
- **LLM Agent UI**: Web-based interface for natural language commands

## Quick Start

### Prerequisites

- Python 3.12+
- MuJoCo physics engine support
- macOS: `mjpython` required for MuJoCo viewer

### Installation

1. Clone the repository:
```bash
git clone https://github.com/ChloePark85/with-robot-4th.git
cd with-robot-4th
```

2. Install dependencies:
```bash
pip install -r requirements.txt
pip install elevenlabs langgraph langchain-openai
```

Dependencies:
- FastAPI 0.121.1
- MuJoCo 3.3.7
- uvicorn 0.38.0
- elevenlabs (for voice commands)
- langgraph, langchain-openai (for LLM agent)

3. **Download Assets (Required, ~5GB)**:

The simulation requires mesh and texture assets from RoboCasa and robosuite projects.

```bash
# Clone RoboCasa and download assets
git clone --depth 1 https://github.com/robocasa/robocasa.git /tmp/robocasa
cd /tmp/robocasa && python robocasa/scripts/download_kitchen_assets.py

# Copy RoboCasa assets
cp -r ~/Desktop/robocasa/robocasa/models/assets/* model/robocasa/assets/

# Copy robosuite robot assets (robots, grippers, bases)
cp -r /path/to/robosuite/robosuite/models/assets/robots model/robocasa/assets/
cp -r /path/to/robosuite/robosuite/models/assets/grippers model/robocasa/assets/
cp -r /path/to/robosuite/robosuite/models/assets/bases model/robocasa/assets/
```

4. **Setup ElevenLabs API Key (for voice commands)**:
```bash
cd agent
cp .env.example .env
# Edit .env and add your ELEVENLABS_API_KEY
```

### Running the Simulator

**macOS (requires mjpython):**
```bash
cd robot
mjpython main.py
```

**Linux:**
```bash
cd robot
python main.py
```

### Running the Agent UI

```bash
cd agent
python3.12 main.py
```

Open http://localhost:8900 for the web UI with voice commands.

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
      "code": "set_target_position(0, 0, PI)"
    }
  }
}
```

### Example Code

Move robot to position (x=0, y=0, theta=π):
```python
set_target_position(0, 0, PI)
```

Move robot to position (x=-0.5, y=0, theta=π) and wait until reached:
```python
set_target_position(-0.5, 0, PI, wait=True)
```

Move robot in sequence:
```python
set_target_position(1.0, 0, 0)
set_target_position(1.0, 1.0, PI/2)
set_target_position(0, 1.0, PI)
```

### Available Functions in Sandbox

- `set_target_position(x, y, theta, wait=True)`: Set robot target position
  - `x`: Target x position in meters
  - `y`: Target y position in meters
  - `theta`: Target orientation in radians
  - `wait`: Block until position reached (default: True)
- `print()`: Print debug messages
- `range()`, `float()`, `time()`: Standard Python builtins
- `PI`: Constant for π (3.14159...)

## Project Structure

```
with-robot-4th/
├── robot/
│   ├── main.py              # FastAPI server and threading orchestration
│   ├── simulator.py         # MuJoCo simulator with PD controller
│   ├── code_repository.py   # Sandboxed code execution layer
│   └── code_knowledge.md    # Robot control API documentation
├── model/
│   └── robocasa/
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

Change the MuJoCo model in `robot/simulator.py`:

```python
# Line 40
xml_path = "../model/robocasa/panda_omron.xml"  # Default
```

## Technical Details

- **Physics Engine**: MuJoCo 3.3.7
- **Control System**: PD controller for mobile base position tracking
- **Actuators**:
  - Position control (kp=1000, kv=100) for Panda arm
  - Velocity control (kv=1000/1500) for mobile base
- **Sensors**: Force/torque sensors on gripper end-effector
- **Threading**: Daemon threads for clean shutdown
- **Safety**: Sandboxed code execution with restricted builtins

## License

See [LICENSE](LICENSE) file for details.

## Contributing

This is a research/educational project. For questions or contributions, please open an issue.

## Acknowledgments

- MuJoCo physics engine by DeepMind
- Panda robot model by Franka Emika
- Kitchen assets from RoboCasa project
