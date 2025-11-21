"""
LLM-Enhanced Robot Control with VOICE CONTROL
Adds speech recognition to the existing LLM system
"""

import queue
import threading
import uvicorn
from fastapi import FastAPI, Response, status
from fastapi.responses import JSONResponse, HTMLResponse
from fastapi.middleware.cors import CORSMiddleware
from simulator import MujocoSimulator
from llm_agent import LLMRobotAgent
import code_repository


# Server configuration
HOST = "0.0.0.0"
PORT = 8800
VERSION = "0.0.3-VOICE"

# FastAPI application
app = FastAPI(
    title="🎤 Voice-Controlled LLM Robot",
    description="Control robot via voice commands (Korean/English)",
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
    try:
        simulator.run()
    except RuntimeError as e:
        if "mjpython" in str(e):
            print("\n⚠️  WARNING: MuJoCo viewer requires mjpython on macOS")
            print("   Simulator disabled - API server running without 3D viewer")
            print("   To enable viewer: run with 'mjpython llm_voice.py'\n")
        else:
            print(f"\n⚠️  Simulator error: {e}\n")
    except Exception as e:
        print(f"\n⚠️  Simulator error: {e}\n")


@app.get("/")
def read_root():
    """Get server info."""
    return {
        "name": "Voice-Controlled LLM Robot",
        "version": VERSION,
        "status": "running",
        "features": ["voice_control", "natural_language", "gpt4_integration"]
    }


@app.post("/send_action")
def receive_action(action: dict):
    """Queue action for execution (original endpoint)."""
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
    """Natural language command endpoint (LLM-powered)."""
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
        print(f"🎤 Command: {user_command}")
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

        # 자동 실행
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
    """Voice-enabled web UI"""
    return """
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🎤 Voice Robot Control</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        .container {
            max-width: 900px;
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
        .voice-badge {
            display: inline-block;
            background: linear-gradient(135deg, #f093fb 0%, #f5576c 100%);
            color: white;
            padding: 8px 20px;
            border-radius: 20px;
            font-weight: bold;
            margin-bottom: 20px;
            animation: pulse 2s infinite;
        }
        @keyframes pulse {
            0%, 100% { transform: scale(1); }
            50% { transform: scale(1.05); }
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
        .input-container {
            position: relative;
            display: flex;
            gap: 10px;
        }
        input {
            flex: 1;
            padding: 15px;
            border: 2px solid #e0e0e0;
            border-radius: 8px;
            font-size: 18px;
            transition: border-color 0.3s;
        }
        input:focus {
            outline: none;
            border-color: #667eea;
        }
        .voice-btn {
            background: linear-gradient(135deg, #f093fb 0%, #f5576c 100%);
            color: white;
            border: none;
            padding: 15px 30px;
            border-radius: 8px;
            font-size: 24px;
            cursor: pointer;
            transition: all 0.3s;
            min-width: 70px;
        }
        .voice-btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 10px 20px rgba(245, 87, 108, 0.4);
        }
        .voice-btn.listening {
            animation: listening 1s infinite;
            background: linear-gradient(135deg, #ff6b6b 0%, #ee5a6f 100%);
        }
        @keyframes listening {
            0%, 100% { transform: scale(1); }
            50% { transform: scale(1.1); }
        }
        button.execute-btn {
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
        button.execute-btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 10px 20px rgba(102, 126, 234, 0.4);
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
        .status.listening { background: #ff6b6b; color: white; }

        .voice-indicator {
            text-align: center;
            margin: 20px 0;
            padding: 15px;
            background: #fff3cd;
            border-radius: 8px;
            display: none;
        }
        .voice-indicator.active {
            display: block;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="voice-badge">🎤 VOICE ENABLED</div>
        <h1>🤖 Voice Robot Control</h1>
        <p class="subtitle">음성 또는 텍스트로 로봇을 제어하세요 (Powered by GPT-4)</p>

        <div class="voice-indicator" id="voiceIndicator">
            <span class="status listening">🎤 듣는 중...</span>
            <p>명령을 말씀해주세요</p>
        </div>

        <div class="examples">
            <strong>💡 예시 명령어 (음성 또는 클릭):</strong><br>
            <span class="example-btn" onclick="setCommand('정사각형으로 움직여줘')">정사각형 움직이기</span>
            <span class="example-btn" onclick="setCommand('원형으로 천천히 돌아줘')">원형 경로</span>
            <span class="example-btn" onclick="setCommand('삼각형 그려줘')">삼각형 그리기</span>
            <span class="example-btn" onclick="setCommand('제자리에서 한바퀴 회전')">제자리 회전</span>
            <span class="example-btn" onclick="setCommand('앞으로 1미터 이동')">직선 이동</span>
        </div>

        <div class="input-group">
            <label for="command">명령어 입력 (또는 🎤 버튼으로 음성 입력):</label>
            <div class="input-container">
                <input type="text" id="command" placeholder="예: 정사각형으로 움직여줘" />
                <button class="voice-btn" id="voiceBtn" onclick="startVoiceRecognition()">🎤</button>
            </div>
        </div>

        <button class="execute-btn" onclick="sendCommand()">🚀 실행하기</button>

        <div id="response" style="display:none;" class="response">
            <div id="status"></div>
            <div id="userCommand"></div>
            <strong>생성된 코드:</strong>
            <pre id="generatedCode"></pre>
        </div>
    </div>

    <script>
        let recognition = null;
        let isListening = false;

        // Initialize Web Speech API
        if ('webkitSpeechRecognition' in window || 'SpeechRecognition' in window) {
            const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
            recognition = new SpeechRecognition();
            recognition.lang = 'ko-KR';  // Korean
            recognition.continuous = false;
            recognition.interimResults = false;

            recognition.onstart = function() {
                isListening = true;
                document.getElementById('voiceBtn').classList.add('listening');
                document.getElementById('voiceIndicator').classList.add('active');
                console.log('🎤 Voice recognition started');
            };

            recognition.onend = function() {
                isListening = false;
                document.getElementById('voiceBtn').classList.remove('listening');
                document.getElementById('voiceIndicator').classList.remove('active');
                console.log('🎤 Voice recognition ended');
            };

            recognition.onresult = function(event) {
                const transcript = event.results[0][0].transcript;
                console.log('🎤 Recognized:', transcript);

                // Fill input field
                document.getElementById('command').value = transcript;

                // Auto-execute
                setTimeout(() => {
                    sendCommand();
                }, 500);
            };

            recognition.onerror = function(event) {
                console.error('🎤 Voice recognition error:', event.error);
                isListening = false;
                document.getElementById('voiceBtn').classList.remove('listening');
                document.getElementById('voiceIndicator').classList.remove('active');

                if (event.error === 'no-speech') {
                    alert('음성이 감지되지 않았습니다. 다시 시도해주세요.');
                } else {
                    alert('음성 인식 오류: ' + event.error);
                }
            };
        } else {
            console.warn('⚠️ Web Speech API not supported in this browser');
        }

        function startVoiceRecognition() {
            if (!recognition) {
                alert('이 브라우저는 음성 인식을 지원하지 않습니다. Chrome을 사용해주세요.');
                return;
            }

            if (isListening) {
                recognition.stop();
            } else {
                recognition.start();
            }
        }

        function setCommand(cmd) {
            document.getElementById('command').value = cmd;
        }

        async function sendCommand() {
            const command = document.getElementById('command').value;
            if (!command.trim()) {
                alert('명령어를 입력하거나 🎤 버튼을 눌러 음성으로 입력해주세요!');
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

        // Enter key support
        document.getElementById('command').addEventListener('keypress', function(e) {
            if (e.key === 'Enter') sendCommand();
        });

        // Show voice support status
        window.addEventListener('load', function() {
            if (!recognition) {
                const subtitle = document.querySelector('.subtitle');
                subtitle.innerHTML += '<br><small style="color: #f44336;">⚠️ 음성 인식을 사용하려면 Chrome 브라우저를 사용하세요</small>';
            }
        });
    </script>
</body>
</html>
    """


def main():
    """Start simulator, action processor, and FastAPI server."""
    # Start background threads
    threading.Thread(target=run_simulator, daemon=True).start()
    threading.Thread(target=process_actions, daemon=True).start()

    # Startup info
    print(f"\n{'='*60}")
    print(f"🎤 Voice-Controlled LLM Robot Simulator")
    print(f"{'='*60}")
    print(f"Server: http://{HOST}:{PORT}")
    print(f"Web UI: http://{HOST}:{PORT}/ui")
    print(f"API docs: http://{HOST}:{PORT}/docs")
    print(f"{'='*60}\n")
    print(f"💡 Features:")
    print(f"  🎤 Voice Control - Speak commands in Korean/English")
    print(f"  ⌨️  Text Input   - Type commands manually")
    print(f"  🤖 LLM Power    - GPT-4 natural language understanding")
    print(f"{'='*60}\n")

    # Start server
    uvicorn.run(app, host=HOST, port=PORT, log_level="info")


if __name__ == "__main__":
    main()
