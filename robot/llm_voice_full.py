"""
Full Voice-Controlled Robot with TTS Responses
- Voice input (STT): Web Speech API
- Voice output (TTS): ElevenLabs
- LLM: GPT-4
"""

import queue
import threading
import uvicorn
from fastapi import FastAPI
from fastapi.responses import JSONResponse, HTMLResponse, FileResponse
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from simulator import MujocoSimulator
from llm_agent import LLMRobotAgent
from tts_agent import TTSAgent
import code_repository
import os


# Server configuration
HOST = "0.0.0.0"
PORT = 8800
VERSION = "0.0.4-VOICE-TTS"

# FastAPI application
app = FastAPI(
    title="🎤🔊 Full Voice Robot Control",
    description="Voice input + Voice output + LLM control",
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

# Create instances
simulator = MujocoSimulator()
code_repository.simulator = simulator
llm_agent = LLMRobotAgent()
tts_agent = TTSAgent()

# Voice feedback for obstacle detection
def obstacle_voice_feedback(message: str):
    """Callback for obstacle detection voice feedback"""
    if tts_agent.enabled:
        print(f"🔊 장애물 음성 피드백: {message}")
        try:
            audio_path = tts_agent.generate_speech(
                text=message,
                output_path="static/obstacle_alert.mp3"
            )
            if audio_path:
                print(f"   ✅ 음성 파일 생성: {audio_path}")
        except Exception as e:
            print(f"   ⚠️ TTS 생성 실패: {e}")

code_repository.voice_callback = obstacle_voice_feedback

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


def run_simulator():
    """Run MuJoCo simulator in background thread."""
    try:
        simulator.run()
    except RuntimeError as e:
        if "mjpython" in str(e):
            print("\n⚠️  WARNING: MuJoCo viewer requires mjpython on macOS")
            print("   Simulator disabled - API server running without 3D viewer")
            print("   To enable viewer: run with 'mjpython llm_voice_full.py'\n")
        else:
            print(f"\n⚠️  Simulator error: {e}\n")
    except Exception as e:
        print(f"\n⚠️  Simulator error: {e}\n")


@app.get("/")
def read_root():
    """Get server info."""
    return {
        "name": "Full Voice Robot Control",
        "version": VERSION,
        "features": ["voice_input", "voice_output", "llm_control"],
        "tts_enabled": tts_agent.enabled
    }


@app.post("/send_action")
def receive_action(action: dict):
    """Queue action for execution."""
    if "action" in action and "type" in action["action"] and "payload" in action["action"]:
        actions_queue.put(action)
        return JSONResponse(
            status_code=200,
            content={"status": "success", "action_feedback": "good"}
        )
    return JSONResponse(
        status_code=400,
        content={"status": "error", "message": "Invalid action format"}
    )


@app.post("/llm_command")
def llm_command(request: dict):
    """
    Natural language command with optional TTS response
    """
    try:
        user_command = request.get("command", "")
        auto_execute = request.get("execute", True)
        enable_tts = request.get("tts", True)

        if not user_command:
            return JSONResponse(
                status_code=400,
                content={"status": "error", "message": "No command provided"}
            )

        # Generate code with LLM
        print(f"\n{'='*60}")
        print(f"🎤 User Command: {user_command}")
        generated_code = llm_agent.generate_code(user_command)
        print(f"Generated Code:\n{generated_code}")

        # Generate TTS response
        audio_url = None
        response_text = None

        if enable_tts and tts_agent.enabled:
            response_text = tts_agent.generate_response(user_command, "execute")
            print(f"🔊 Robot says: {response_text}")

            audio_path = tts_agent.generate_speech(
                text=response_text,
                output_path="static/robot_speech.mp3"
            )

            if audio_path:
                audio_url = "/audio/robot_speech.mp3"

        print(f"{'='*60}\n")

        # Check for errors
        if generated_code.startswith("# ERROR"):
            return JSONResponse(
                status_code=500,
                content={
                    "status": "error",
                    "user_command": user_command,
                    "error": generated_code
                }
            )

        # Execute code
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
            status_code=200,
            content={
                "status": "success",
                "user_command": user_command,
                "generated_code": generated_code,
                "executed": executed,
                "tts_enabled": tts_agent.enabled,
                "audio_url": audio_url,
                "response_text": response_text
            }
        )

    except Exception as e:
        import traceback
        return JSONResponse(
            status_code=500,
            content={
                "status": "error",
                "message": str(e),
                "traceback": traceback.format_exc()
            }
        )


@app.get("/audio/{filename}")
def get_audio(filename: str):
    """Serve audio files"""
    file_path = f"static/{filename}"
    if os.path.exists(file_path):
        return FileResponse(file_path, media_type="audio/mpeg")
    return JSONResponse(status_code=404, content={"error": "File not found"})


@app.get("/ui", response_class=HTMLResponse)
def get_ui():
    """Full voice-enabled UI with TTS"""
    tts_status = "✅ ENABLED" if tts_agent.enabled else "❌ DISABLED (Set ELEVENLABS_API_KEY)"

    return f"""
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🎤🔊 Voice Robot</title>
    <style>
        * {{ margin: 0; padding: 0; box-sizing: border-box; }}
        body {{
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }}
        .container {{
            max-width: 900px;
            margin: 0 auto;
            background: white;
            border-radius: 20px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.3);
            padding: 40px;
        }}
        h1 {{ color: #667eea; margin-bottom: 10px; font-size: 2.5em; }}
        .subtitle {{ color: #666; margin-bottom: 20px; font-size: 1.1em; }}
        .badges {{ margin-bottom: 20px; }}
        .badge {{
            display: inline-block;
            padding: 8px 20px;
            border-radius: 20px;
            font-weight: bold;
            margin: 5px;
            animation: pulse 2s infinite;
        }}
        .badge.voice {{ background: linear-gradient(135deg, #f093fb 0%, #f5576c 100%); color: white; }}
        .badge.tts {{ background: linear-gradient(135deg, #4facfe 0%, #00f2fe 100%); color: white; }}
        @keyframes pulse {{
            0%, 100% {{ transform: scale(1); }}
            50% {{ transform: scale(1.05); }}
        }}
        .input-container {{
            display: flex;
            gap: 10px;
            margin-bottom: 15px;
        }}
        input {{
            flex: 1;
            padding: 15px;
            border: 2px solid #e0e0e0;
            border-radius: 8px;
            font-size: 18px;
        }}
        .voice-btn {{
            background: linear-gradient(135deg, #f093fb 0%, #f5576c 100%);
            color: white;
            border: none;
            padding: 15px 30px;
            border-radius: 8px;
            font-size: 24px;
            cursor: pointer;
            min-width: 70px;
        }}
        .voice-btn.listening {{
            animation: listening 1s infinite;
        }}
        @keyframes listening {{
            0%, 100% {{ transform: scale(1); }}
            50% {{ transform: scale(1.1); }}
        }}
        button.execute-btn {{
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            border: none;
            padding: 15px;
            border-radius: 8px;
            font-size: 18px;
            font-weight: 600;
            cursor: pointer;
            width: 100%;
        }}
        .examples {{
            margin: 20px 0;
            padding: 15px;
            background: #f5f5f5;
            border-radius: 8px;
        }}
        .example-btn {{
            display: inline-block;
            margin: 5px;
            padding: 8px 15px;
            background: white;
            border: 1px solid #ddd;
            border-radius: 5px;
            cursor: pointer;
            font-size: 14px;
        }}
        .example-btn:hover {{ background: #667eea; color: white; }}
        .response {{
            margin-top: 20px;
            padding: 20px;
            background: #f9f9f9;
            border-radius: 8px;
            border-left: 4px solid #667eea;
            display: none;
        }}
        .robot-response {{
            margin: 15px 0;
            padding: 15px;
            background: linear-gradient(135deg, #4facfe 0%, #00f2fe 100%);
            color: white;
            border-radius: 8px;
            display: flex;
            align-items: center;
            gap: 10px;
        }}
        pre {{
            background: #2d2d2d;
            color: #f8f8f2;
            padding: 15px;
            border-radius: 8px;
            overflow-x: auto;
            margin-top: 10px;
        }}
        .status {{ padding: 5px 10px; border-radius: 5px; font-weight: 600; }}
        .status.success {{ background: #4caf50; color: white; }}
        .tts-status {{ font-size: 0.9em; color: #666; margin-bottom: 20px; }}
    </style>
</head>
<body>
    <div class="container">
        <div class="badges">
            <div class="badge voice">🎤 VOICE INPUT</div>
            <div class="badge tts">🔊 VOICE OUTPUT</div>
        </div>
        <h1>🤖 Voice Robot Control</h1>
        <p class="subtitle">로봇과 대화하세요! (Powered by GPT-4 + ElevenLabs)</p>
        <p class="tts-status">TTS Status: {tts_status}</p>

        <div class="examples">
            <strong>💡 말해보세요:</strong><br>
            <span class="example-btn" onclick="setCommand('정사각형으로 움직여줘')">정사각형</span>
            <span class="example-btn" onclick="setCommand('원형으로 천천히 돌아줘')">원형</span>
            <span class="example-btn" onclick="setCommand('삼각형 그려줘')">삼각형</span>
            <span class="example-btn" onclick="setCommand('제자리에서 회전')">회전</span>
        </div>

        <div class="input-container">
            <input type="text" id="command" placeholder="명령을 입력하거나 🎤 버튼을 누르세요..." />
            <button class="voice-btn" id="voiceBtn" onclick="startVoice()">🎤</button>
        </div>

        <button class="execute-btn" onclick="sendCommand()">🚀 실행하기</button>

        <div id="response" class="response">
            <div id="status"></div>
            <div id="robotResponse" style="display:none;" class="robot-response">
                <span style="font-size: 24px;">🤖</span>
                <div>
                    <strong>로봇 응답:</strong>
                    <p id="robotText"></p>
                    <audio id="robotAudio" controls style="margin-top: 10px; width: 100%;"></audio>
                </div>
            </div>
            <div id="userCommand"></div>
            <strong>생성된 코드:</strong>
            <pre id="generatedCode"></pre>
        </div>
    </div>

    <script>
        let recognition = null;
        let isListening = false;

        // Initialize Speech Recognition
        if ('webkitSpeechRecognition' in window || 'SpeechRecognition' in window) {{
            const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
            recognition = new SpeechRecognition();
            recognition.lang = 'ko-KR';
            recognition.continuous = false;
            recognition.interimResults = false;

            recognition.onstart = () => {{
                isListening = true;
                document.getElementById('voiceBtn').classList.add('listening');
            }};

            recognition.onend = () => {{
                isListening = false;
                document.getElementById('voiceBtn').classList.remove('listening');
            }};

            recognition.onresult = (event) => {{
                const transcript = event.results[0][0].transcript;
                document.getElementById('command').value = transcript;
                setTimeout(() => sendCommand(), 500);
            }};

            recognition.onerror = (event) => {{
                console.error('Recognition error:', event.error);
                isListening = false;
                document.getElementById('voiceBtn').classList.remove('listening');
            }};
        }}

        function startVoice() {{
            if (!recognition) {{
                alert('이 브라우저는 음성 인식을 지원하지 않습니다. Chrome을 사용해주세요.');
                return;
            }}
            if (isListening) {{
                recognition.stop();
            }} else {{
                recognition.start();
            }}
        }}

        function setCommand(cmd) {{
            document.getElementById('command').value = cmd;
        }}

        async function sendCommand() {{
            const command = document.getElementById('command').value;
            if (!command.trim()) {{
                alert('명령어를 입력하거나 🎤 버튼을 누르세요!');
                return;
            }}

            const responseDiv = document.getElementById('response');
            const statusDiv = document.getElementById('status');
            const robotResponseDiv = document.getElementById('robotResponse');
            const robotTextDiv = document.getElementById('robotText');
            const robotAudioElement = document.getElementById('robotAudio');
            const userCommandDiv = document.getElementById('userCommand');
            const codeDiv = document.getElementById('generatedCode');

            // Show loading
            responseDiv.style.display = 'block';
            statusDiv.innerHTML = '<span class="status">⏳ 처리 중...</span>';
            robotResponseDiv.style.display = 'none';
            userCommandDiv.textContent = '';
            codeDiv.textContent = '';

            try {{
                const response = await fetch('/llm_command', {{
                    method: 'POST',
                    headers: {{ 'Content-Type': 'application/json' }},
                    body: JSON.stringify({{ command: command, execute: true, tts: true }})
                }});

                const data = await response.json();

                if (data.status === 'success') {{
                    statusDiv.innerHTML = '<span class="status success">✅ 성공!</span>';
                    userCommandDiv.innerHTML = `<strong>명령어:</strong> ${{data.user_command}}`;
                    codeDiv.textContent = data.generated_code;

                    // Show robot voice response
                    if (data.audio_url && data.response_text) {{
                        robotResponseDiv.style.display = 'flex';
                        robotTextDiv.textContent = data.response_text;
                        robotAudioElement.src = data.audio_url;
                        robotAudioElement.play();
                    }}
                }} else {{
                    statusDiv.innerHTML = '<span class="status error">❌ 오류</span>';
                    userCommandDiv.innerHTML = `<strong>오류:</strong> ${{data.message || data.error}}`;
                }}
            }} catch (error) {{
                statusDiv.innerHTML = '<span class="status error">❌ 연결 오류</span>';
                userCommandDiv.innerHTML = `<strong>오류:</strong> ${{error.message}}`;
            }}
        }}

        document.getElementById('command').addEventListener('keypress', (e) => {{
            if (e.key === 'Enter') sendCommand();
        }});
    </script>
</body>
</html>
    """


def main():
    """Start simulator, action processor, and FastAPI server."""
    # Create static directory for audio files
    os.makedirs("static", exist_ok=True)

    # Start background threads
    threading.Thread(target=run_simulator, daemon=True).start()
    threading.Thread(target=process_actions, daemon=True).start()

    # Startup info
    print(f"\n{'='*60}")
    print(f"🎤🔊 Full Voice-Controlled Robot Simulator")
    print(f"{'='*60}")
    print(f"Server: http://{HOST}:{PORT}")
    print(f"Web UI: http://{HOST}:{PORT}/ui")
    print(f"{'='*60}\n")
    print(f"💡 Features:")
    print(f"  🎤 Voice Input  - Speak commands")
    print(f"  🔊 Voice Output - Robot speaks back (ElevenLabs)")
    print(f"  🤖 LLM Power    - GPT-4 understanding")
    print(f"\n  TTS: {'✅ Enabled' if tts_agent.enabled else '❌ Disabled (set ELEVENLABS_API_KEY)'}")
    print(f"{'='*60}\n")

    # Start server
    uvicorn.run(app, host=HOST, port=PORT, log_level="info")


if __name__ == "__main__":
    main()
