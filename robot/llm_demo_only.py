"""
LLM Demo - Code generation only (no robot execution)
For quick demos focused on LLM capabilities
"""

import uvicorn
from fastapi import FastAPI
from fastapi.responses import JSONResponse, HTMLResponse
from fastapi.middleware.cors import CORSMiddleware
from llm_agent import LLMRobotAgent

HOST = "0.0.0.0"
PORT = 8800
VERSION = "0.0.3-DEMO"

app = FastAPI(
    title="LLM Robot Control Demo",
    description="Natural language to code generation (demo mode)",
    version=VERSION
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

llm_agent = LLMRobotAgent()


@app.get("/")
def read_root():
    return {
        "name": "LLM Robot Control Demo",
        "version": VERSION,
        "mode": "code_generation_only"
    }


@app.post("/llm_command")
def llm_command(request: dict):
    """Generate code from natural language (demo mode)"""
    try:
        user_command = request.get("command", "")

        if not user_command:
            return JSONResponse(
                status_code=400,
                content={"status": "error", "message": "No command provided"}
            )

        # Generate code
        generated_code = llm_agent.generate_code(user_command)

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

        # Return generated code (no execution in demo mode)
        return JSONResponse(
            status_code=200,
            content={
                "status": "success",
                "user_command": user_command,
                "generated_code": generated_code,
                "executed": False,
                "note": "Demo mode - code generation only"
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


@app.get("/ui", response_class=HTMLResponse)
def get_ui():
    """Demo UI"""
    return """
<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <title>🤖 LLM Robot Demo</title>
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
        h1 { color: #667eea; margin-bottom: 10px; font-size: 2.5em; }
        .subtitle { color: #666; margin-bottom: 30px; font-size: 1.1em; }
        .demo-badge {
            display: inline-block;
            background: #ffd700;
            color: #333;
            padding: 5px 15px;
            border-radius: 20px;
            font-weight: bold;
            margin-bottom: 20px;
        }
        input {
            width: 100%;
            padding: 15px;
            border: 2px solid #e0e0e0;
            border-radius: 8px;
            font-size: 18px;
            margin-bottom: 15px;
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
        }
        button:hover { transform: translateY(-2px); }
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
        }
        .example-btn:hover { background: #667eea; color: white; }
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
        .status { padding: 5px 10px; border-radius: 5px; font-weight: 600; }
        .status.success { background: #4caf50; color: white; }
    </style>
</head>
<body>
    <div class="container">
        <div class="demo-badge">🎬 DEMO MODE</div>
        <h1>🤖 LLM Robot Control</h1>
        <p class="subtitle">자연어 → Python 코드 자동 생성 (Powered by GPT-4)</p>

        <div class="examples">
            <strong>💡 예시 명령어:</strong><br>
            <span class="example-btn" onclick="setCommand('정사각형으로 움직여줘')">정사각형</span>
            <span class="example-btn" onclick="setCommand('원형으로 천천히 돌아줘')">원형</span>
            <span class="example-btn" onclick="setCommand('삼각형 그려줘')">삼각형</span>
            <span class="example-btn" onclick="setCommand('제자리에서 회전')">회전</span>
        </div>

        <input type="text" id="command" placeholder="명령어를 입력하세요..." />
        <button onclick="sendCommand()">🚀 코드 생성</button>

        <div id="response" style="display:none;" class="response">
            <span class="status success">✅ 성공!</span>
            <p><strong>명령어:</strong> <span id="userCommand"></span></p>
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

            const response = await fetch('/llm_command', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ command: command, execute: false })
            });

            const data = await response.json();

            if (data.status === 'success') {
                document.getElementById('response').style.display = 'block';
                document.getElementById('userCommand').textContent = data.user_command;
                document.getElementById('generatedCode').textContent = data.generated_code;
            }
        }

        document.getElementById('command').addEventListener('keypress', function(e) {
            if (e.key === 'Enter') sendCommand();
        });
    </script>
</body>
</html>
    """


if __name__ == "__main__":
    print("\n" + "="*60)
    print("🎬 LLM Robot Control DEMO")
    print("="*60)
    print(f"Server: http://{HOST}:{PORT}")
    print(f"Web UI: http://{HOST}:{PORT}/ui")
    print("="*60)
    print("\n💡 Demo Mode: Code generation only (no robot execution)")
    print("   Focus on showing LLM capabilities\n")

    uvicorn.run(app, host=HOST, port=PORT, log_level="info")
