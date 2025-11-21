"""
LLM-Enhanced FastAPI server for MuJoCo robot simulation
Extends main.py with natural language control capabilities
"""

import queue
import threading
import uvicorn
from fastapi import FastAPI, Response, status
from fastapi.responses import JSONResponse, HTMLResponse
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
from simulator import MujocoSimulator
from llm_agent import LLMRobotAgent
import code_repository

# Load environment variables from .env file
load_dotenv()


# Server configuration
HOST = "0.0.0.0"
PORT = 8800  # Change to 8801 if port conflict
VERSION = "0.0.2-LLM"

# FastAPI application
app = FastAPI(
    title="LLM-Enabled MuJoCo Robot Simulator",
    description="Control Panda-Omron mobile robot via natural language (GPT-4)",
    version=VERSION
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Create simulator and LLM agent
simulator = MujocoSimulator()
code_repository.simulator = simulator
llm_agent = LLMRobotAgent()

# Action queue
actions_queue = queue.Queue()


def process_actions():
    """Process action queue in background thread."""
    print("Action processor started...")
    while True:
        try:
            action = actions_queue.get(timeout=0.1)
            action = action["action"]

            print(f"\n{'='*60}")
            print(f"Received Action:", action)

            if action["type"] == "run_code":
                code_str = action["payload"].get("code")
                try:
                    code_repository.exec_code(code_str)
                    print("Code execution completed successfully")
                except Exception as e:
                    print(f"\n[EXECUTION ERROR]")
                    print(f"  Type: {type(e).__name__}")
                    print(f"  Message: {e}")
                    import traceback
                    traceback.print_exc()
            print(f"{'='*60}\n")

            actions_queue.task_done()

        except queue.Empty:
            continue
        except Exception as e:
            print(f"Error processing action: {e}")
            import traceback
            traceback.print_exc()


def run_simulator():
    """Run MuJoCo simulator in background thread."""
    print("🎬 Simulator thread starting...")
    try:
        print("🎬 Calling simulator.run() - this will open MuJoCo viewer")
        simulator.run()
        print("🎬 simulator.run() returned (viewer closed)")
    except RuntimeError as e:
        if "mjpython" in str(e):
            print("\n⚠️  WARNING: MuJoCo viewer requires mjpython on macOS")
            print("   Simulator disabled - API server running without 3D viewer")
            print("   To enable viewer: run with 'mjpython llm_main.py'\n")
        else:
            print(f"\n⚠️  Simulator error: {e}\n")
    except Exception as e:
        print(f"\n⚠️  Simulator error: {e}\n")
        import traceback
        traceback.print_exc()


@app.get("/")
def read_root():
    """Get server info."""
    return {
        "name": "LLM-Enabled MuJoCo Robot Simulator",
        "version": VERSION,
        "status": "running",
        "features": ["natural_language_control", "gpt4_integration"]
    }


@app.post("/send_action")
def receive_action(action: dict):
    """
    Queue action for execution (original endpoint).

    Expected format:
        {
            "action": {
                "type": "run_code",
                "payload": {"code": "set_target_position(0, 0, PI)"}
            }
        }
    """
    if "action" in action and "type" in action["action"] and "payload" in action["action"]:
        actions_queue.put(action)
        return JSONResponse(
            status_code=status.HTTP_200_OK,
            content={"status": "success", "action_feedback": "good"}
        )
    return JSONResponse(
        status_code=status.HTTP_400_BAD_REQUEST,
        content={"status": "error", "message": "Invalid action format"}
    )


@app.post("/llm_command")
def llm_command(request: dict):
    """
    🤖 NEW: Natural language command endpoint (LLM-powered)

    Expected format:
        {
            "command": "정사각형으로 움직여줘",
            "execute": true  # Optional: auto-execute generated code
        }

    Returns:
        {
            "status": "success",
            "user_command": "...",
            "generated_code": "...",
            "executed": true/false
        }
    """
    try:
        user_command = request.get("command", "")
        auto_execute = request.get("execute", True)

        if not user_command:
            return JSONResponse(
                status_code=status.HTTP_400_BAD_REQUEST,
                content={"status": "error", "message": "No command provided"}
            )

        # LLM으로 코드 생성
        print(f"\n{'='*60}")
        print(f"🤖 LLM Command: {user_command}")
        generated_code = llm_agent.generate_code(user_command)
        print(f"Generated Code:\n{generated_code}")
        print(f"{'='*60}\n")

        # 에러 체크
        if generated_code.startswith("# ERROR"):
            return JSONResponse(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                content={
                    "status": "error",
                    "user_command": user_command,
                    "error": generated_code
                }
            )

        # 자동 실행 옵션
        executed = False
        if auto_execute:
            actions_queue.put({
                "action": {
                    "type": "run_code",
                    "payload": {"code": generated_code}
                }
            })
            executed = True

        return JSONResponse(
            status_code=status.HTTP_200_OK,
            content={
                "status": "success",
                "user_command": user_command,
                "generated_code": generated_code,
                "executed": executed
            }
        )

    except Exception as e:
        import traceback
        return JSONResponse(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            content={
                "status": "error",
                "message": str(e),
                "traceback": traceback.format_exc()
            }
        )


@app.get("/ui", response_class=HTMLResponse)
def get_ui():
    """Simple web UI for LLM robot control"""
    return """
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🤖 LLM Robot Control</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        .container {
            max-width: 800px;
            margin: 0 auto;
            background: white;
            border-radius: 20px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.3);
            padding: 40px;
        }
        h1 {
            color: #667eea;
            margin-bottom: 10px;
            font-size: 2.5em;
        }
        .subtitle {
            color: #666;
            margin-bottom: 30px;
            font-size: 1.1em;
        }
        .input-group {
            margin-bottom: 20px;
        }
        label {
            display: block;
            margin-bottom: 8px;
            font-weight: 600;
            color: #333;
        }
        input, textarea {
            width: 100%;
            padding: 12px;
            border: 2px solid #e0e0e0;
            border-radius: 8px;
            font-size: 16px;
            transition: border-color 0.3s;
        }
        input:focus, textarea:focus {
            outline: none;
            border-color: #667eea;
        }
        button {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            border: none;
            padding: 15px 30px;
            border-radius: 8px;
            font-size: 18px;
            font-weight: 600;
            cursor: pointer;
            width: 100%;
            transition: transform 0.2s, box-shadow 0.2s;
        }
        button:hover {
            transform: translateY(-2px);
            box-shadow: 0 10px 20px rgba(102, 126, 234, 0.4);
        }
        button:active {
            transform: translateY(0);
        }
        .examples {
            margin: 20px 0;
            padding: 15px;
            background: #f5f5f5;
            border-radius: 8px;
        }
        .example-btn {
            display: inline-block;
            margin: 5px;
            padding: 8px 15px;
            background: white;
            border: 1px solid #ddd;
            border-radius: 5px;
            cursor: pointer;
            font-size: 14px;
            transition: all 0.2s;
        }
        .example-btn:hover {
            background: #667eea;
            color: white;
            border-color: #667eea;
        }
        .response {
            margin-top: 20px;
            padding: 20px;
            background: #f9f9f9;
            border-radius: 8px;
            border-left: 4px solid #667eea;
        }
        pre {
            background: #2d2d2d;
            color: #f8f8f2;
            padding: 15px;
            border-radius: 8px;
            overflow-x: auto;
            margin-top: 10px;
        }
        .status {
            display: inline-block;
            padding: 5px 10px;
            border-radius: 5px;
            font-weight: 600;
            margin-bottom: 10px;
        }
        .status.success { background: #4caf50; color: white; }
        .status.error { background: #f44336; color: white; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 LLM Robot Control</h1>
        <p class="subtitle">자연어로 로봇을 제어하세요 (Powered by GPT-4)</p>

        <div class="examples">
            <strong>💡 예시 명령어:</strong><br>
            <span class="example-btn" onclick="setCommand('정사각형으로 움직여줘')">정사각형 움직이기</span>
            <span class="example-btn" onclick="setCommand('원형으로 천천히 돌아줘')">원형 경로</span>
            <span class="example-btn" onclick="setCommand('삼각형 그려줘')">삼각형 그리기</span>
            <span class="example-btn" onclick="setCommand('제자리에서 한바퀴 회전')">제자리 회전</span>
            <span class="example-btn" onclick="setCommand('앞으로 1미터 이동')">직선 이동</span>
        </div>

        <div class="input-group">
            <label for="command">명령어 입력:</label>
            <input type="text" id="command" placeholder="예: 정사각형으로 움직여줘" />
        </div>

        <button onclick="sendCommand()">🚀 실행하기</button>

        <div id="response" style="display:none;" class="response">
            <div id="status"></div>
            <div id="userCommand"></div>
            <strong>생성된 코드:</strong>
            <pre id="generatedCode"></pre>
        </div>
    </div>

    <script>
        function setCommand(cmd) {
            document.getElementById('command').value = cmd;
        }

        async function sendCommand() {
            const command = document.getElementById('command').value;
            if (!command.trim()) {
                alert('명령어를 입력해주세요!');
                return;
            }

            const responseDiv = document.getElementById('response');
            const statusDiv = document.getElementById('status');
            const userCommandDiv = document.getElementById('userCommand');
            const codeDiv = document.getElementById('generatedCode');

            // Show loading
            responseDiv.style.display = 'block';
            statusDiv.innerHTML = '<span class="status">⏳ 처리 중...</span>';
            userCommandDiv.textContent = '';
            codeDiv.textContent = '';

            try {
                const response = await fetch('/llm_command', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ command: command, execute: true })
                });

                const data = await response.json();

                if (data.status === 'success') {
                    statusDiv.innerHTML = '<span class="status success">✅ 성공!</span>';
                    userCommandDiv.innerHTML = `<strong>명령어:</strong> ${data.user_command}`;
                    codeDiv.textContent = data.generated_code;
                } else {
                    statusDiv.innerHTML = '<span class="status error">❌ 오류</span>';
                    userCommandDiv.innerHTML = `<strong>오류:</strong> ${data.message || data.error}`;
                }
            } catch (error) {
                statusDiv.innerHTML = '<span class="status error">❌ 연결 오류</span>';
                userCommandDiv.innerHTML = `<strong>오류:</strong> ${error.message}`;
            }
        }

        // Enter 키로 전송
        document.getElementById('command').addEventListener('keypress', function(e) {
            if (e.key === 'Enter') sendCommand();
        });
    </script>
</body>
</html>
    """


def main():
    """
    Start simulator, action processor, and FastAPI server.
    """
    # Start background threads
    print("🔧 Starting background threads...")
    sim_thread = threading.Thread(target=run_simulator, daemon=True)
    action_thread = threading.Thread(target=process_actions, daemon=True)
    
    sim_thread.start()
    action_thread.start()
    
    # Give threads time to initialize
    import time
    time.sleep(0.5)
    
    print("🔧 Threads started. Checking status...")
    print(f"   Simulator thread alive: {sim_thread.is_alive()}")
    print(f"   Action thread alive: {action_thread.is_alive()}")

    # Startup info
    print(f"\n{'='*60}")
    print(f"🤖 LLM-Enabled MuJoCo Robot Simulator")
    print(f"{'='*60}")
    print(f"Server: http://{HOST}:{PORT}")
    print(f"Web UI: http://{HOST}:{PORT}/ui")
    print(f"API docs: http://{HOST}:{PORT}/docs")
    print(f"{'='*60}\n")
    print(f"💡 Endpoints:")
    print(f"  POST /llm_command  - Natural language control (GPT-4)")
    print(f"  POST /send_action  - Direct code execution")
    print(f"  GET  /ui           - Web interface")
    print(f"{'='*60}\n")

    # Start server
    uvicorn.run(app, host=HOST, port=PORT, log_level="info")


if __name__ == "__main__":
    main()
