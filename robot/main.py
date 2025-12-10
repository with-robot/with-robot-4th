"""FastAPI server for MuJoCo robot simulation with REST API control."""

import time
import queue
import threading
import uvicorn
from typing import Dict, Any, Optional
from fastapi import FastAPI, Response, status
from simulator import MujocoSimulator
import code_repository


# Server configuration
HOST = "0.0.0.0"  # Listen on all network interfaces
PORT = 8800       # API server port
VERSION = "0.0.1"

# FastAPI application instance
app = FastAPI(
    title="MuJoCo Robot Simulator API",
    description="Control Panda-Omron mobile robot via REST API",
    version=VERSION
)

# Create simulator instance and inject into code_repository
simulator = MujocoSimulator()
code_repository.simulator = simulator


def process_actions(action: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """Process action."""
    RESULT = {}
    if action["type"] == "run_code":
        code_str = action["payload"].get("code")
        try:
            RESULT = code_repository.exec_code(code_str)
            print(f"Code execution completed: {RESULT}")
        except Exception as e:
            # Log errors without crashing the simulator
            print(f"\n[EXECUTION ERROR]")
            print(f"  Type: {type(e).__name__}")
            print(f"  Message: {e}")
            import traceback
            print(f"\n[TRACEBACK]")
            traceback.print_exc()
    print("=" * 60 + "\n")
    return RESULT


def run_simulator() -> None:
    """Run MuJoCo simulator in background thread."""
    simulator.run()


@app.get("/")
def read_root() -> Dict[str, str]:
    """Get server info."""
    return {"name": "MuJoCo Robot Simulator", "version": VERSION, "status": "running"}


@app.get("/env")
def get_environment():
    """Collect environment snapshot with object poses and robot state."""
    objects = simulator.get_object_positions()
    for obj in objects.values():
        obj['pos'] = obj['pos'].tolist()
        obj['ori'] = obj['ori'].tolist()
    return {
        "timestamp": time.time(),
        "objects": objects,
    }


@app.post("/send_action")
def receive_action(payload: Dict[str, Any]) -> Dict[str, Any]:
    """
    Queue action for execution.

    Expected format:
        {
            "action": {
                "type": "run_code",
                "payload": {"code": "get_mobile_target_joint([0, 0, PI])"}
            }
        }
    """
    # Validate action format
    if "action" in payload and "type" in payload["action"] and "payload" in payload["action"]:
        RESULT = process_actions(payload["action"])
        return {"status": "success", "result": RESULT}
    
    return {"status": "error", "message": "Invalid action format"}


def main() -> None:
    """
    Start simulator and FastAPI server.

    Creates three concurrent threads:
        1. Main thread: FastAPI uvicorn server
        2. Simulator thread: MuJoCo physics simulation with 3D viewer
    """
    # Start background threads (daemon=True ensures cleanup on exit)
    threading.Thread(target=run_simulator, daemon=True).start()

    # Display startup information
    print("\n" + "=" * 60)
    print(f"MuJoCo Robot Simulator API")
    print("=" * 60)
    print(f"Server: http://{HOST}:{PORT}")
    print(f"API docs: http://{HOST}:{PORT}/docs")
    print("=" * 60 + "\n")

    # Start FastAPI server (blocking call)
    uvicorn.run(app, host=HOST, port=PORT, log_level="info")


if __name__ == "__main__":
    main()
