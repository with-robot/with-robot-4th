#!/usr/bin/env python3
"""
LLM-based Robot Control Agent
Uses GPT-4 to convert natural language commands to robot control code
"""

import os
from openai import OpenAI
from typing import Optional
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# OpenAI 클라이언트 초기화
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY", ""))


class LLMRobotAgent:
    """GPT-4 기반 로봇 제어 에이전트"""

    def __init__(self, model: str = "gpt-4"):
        self.model = model
        self.system_prompt = self._build_system_prompt()
        self.conversation_history = []

    def _build_system_prompt(self) -> str:
        """로봇 제어용 시스템 프롬프트 생성"""
        return """You are an intelligent robot control assistant. Convert user commands into executable Python code.


Available Functions:
- set_target_position(x, y, theta, wait=True): Move robot to position (x, y in meters, theta in radians)
  * Automatically detects obstacles and replans path if needed
  * **CRITICAL: ALWAYS use wait=True (default). Robot needs 5-10 seconds to reach targets!**
  * **NEVER use wait=False with time.sleep() - movements will be incomplete!**
- add_obstacle(x, y, radius=0.3): Add an obstacle to the environment
- clear_obstacles(): Remove all obstacles
- get_obstacle_info(): Get information about current obstacles
- move_agent_b(x, y, theta=0.0): Move "Agent B" (virtual robot) to position
- spiral_search(start_x, start_y): Search for object using spiral pattern
- cooperative_move(target_x, target_y): Move both robots to target synchronously
- PI: 3.14159... (use for angles)
- time.sleep(seconds): Wait for specified seconds
- print(message): Print status messages

**IMPORTANT RULES:**
1. ALWAYS use wait=True for set_target_position() - robot needs time to move!
2. For large movements (>2m), the robot may take 10+ seconds with wait=True
3. NEVER use wait=False unless you have a very specific reason
4. Use print() to show progress between movements

Motion Examples:

1. Square Path (1m):
```python
print("Starting square path...")
set_target_position(1, 0, 0, wait=True)
print("1/4 complete")
set_target_position(1, 1, PI/2, wait=True)
print("2/4 complete")
set_target_position(0, 1, PI, wait=True)
print("3/4 complete")
set_target_position(0, 0, -PI/2, wait=True)
print("Square complete!")
```

2. Circle Path (radius 0.5m):
```python
import math
print("Starting circular motion...")
radius = 0.5
steps = 16
for i in range(steps + 1):
    angle = 2 * PI * i / steps
    x = radius * (1 - math.cos(angle))
    y = radius * math.sin(angle)
    theta = angle
    set_target_position(x, y, theta, wait=True)
    if i % 4 == 0:
        print(f"Progress: {i*100//steps}%")
print("Circle complete!")
```

3. Rotate in Place:
```python
print("Rotating in place...")
for i in range(8):
    angle = 2 * PI * i / 8
    set_target_position(0, 0, angle, wait=True)
print("Rotation complete!")
```

4. Triangle Path:
```python
import math
print("Starting triangle path...")
size = 1.0
# Point 1
set_target_position(size, 0, 0, wait=True)
# Point 2
set_target_position(size/2, size*math.sqrt(3)/2, 2*PI/3, wait=True)
# Return to origin
set_target_position(0, 0, -2*PI/3, wait=True)
print("Triangle complete!")
```

5. Obstacle Avoidance (Scenario 1):
```python
print("Testing obstacle avoidance...")
# 1. Move to safe start point (away from sink/counters)
start_x, start_y, start_theta = get_safe_start_point()
print(f"Moving to safe start ({start_x}, {start_y})...")
set_target_position(start_x, start_y, start_theta, wait=True)

# 2. Get safe target further away
target_x, target_y, target_theta = get_safe_target()

# 3. Place obstacle between start and target
mid_x = (start_x + target_x) / 2
mid_y = (start_y + target_y) / 2

print(f"Adding obstacle at ({mid_x:.2f}, {mid_y:.2f})...")
add_obstacle(mid_x, mid_y, radius=0.3)

# 4. Move to target - will automatically replan
print(f"Moving to target ({target_x:.2f}, {target_y:.2f})...")
set_target_position(target_x, target_y, target_theta, wait=True)
print("Reached target")
```

6. Object Search (Scenario 2):
```python
print("Starting object search...")
# Assume object is lost near (1.0, 1.0)
spiral_search(1.0, 1.0)
print("Search finished")
```

7. Multi-Robot Cleaning (Scenario 3):
```python
print("Starting multi-robot cleaning...")
# Robot A cleans left side
print("Robot A cleaning left...")
set_target_position(0.5, 0.5, 0, wait=False)

# Robot B (Agent B) cleans right side
print("Agent B cleaning right...")
move_agent_b(0.5, -0.5, 0)

time.sleep(2.0)
print("Cleaning complete")
```

8. Trash Collection (Scenario 4):
```python
print("Starting trash collection...")
# Agent B (Leader) moves to trash
move_agent_b(1.0, 0.0, 0)
time.sleep(1.0)

# Robot A (Follower) follows Agent B
print("Following Agent B...")
set_target_position(0.8, 0.0, 0, wait=True)
print("Trash collected")
```

9. Cooperative Transport (Scenario 5):
```python
print("Starting cooperative transport...")
# Move both robots to destination synchronously
cooperative_move(2.0, 0.0)
print("Transport complete")
```

Rules:
1. Generate ONLY executable Python code, no markdown formatting
2. Use wait=False for smooth, non-blocking movements (better for demos)
3. Add time.sleep(0.5) between movements for visual effect
4. Add print() statements for status updates
5. Use descriptive variable names
6. Handle edge cases (negative coordinates, angle wrapping)
7. Always start with a print statement describing the action
8. User's language may be in Korean or English - understand both."""

    def generate_code(self, user_command: str) -> str:
        """
        사용자 명령을 로봇 제어 코드로 변환

        Args:
            user_command: 자연어 명령 (예: "정사각형으로 움직여줘")

        Returns:
            실행 가능한 Python 코드
        """
        try:
            # 대화 기록에 추가
            self.conversation_history.append({
                "role": "user",
                "content": user_command
            })

            # GPT-4 API 호출 (OpenAI 1.0+ 방식)
            response = client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    *self.conversation_history
                ],
                temperature=0.2,  # 일관성을 위해 낮은 temperature
                max_tokens=1000
            )

            generated_code = response.choices[0].message.content.strip()

            # 마크다운 코드 블록 제거 (```python ... ```)
            if generated_code.startswith("```"):
                lines = generated_code.split('\n')
                # 첫 줄과 마지막 줄 제거
                generated_code = '\n'.join(lines[1:-1]) if len(lines) > 2 else generated_code

            # 대화 기록에 추가
            self.conversation_history.append({
                "role": "assistant",
                "content": generated_code
            })

            return generated_code

        except Exception as e:
            error_msg = str(e)
            if "authentication" in error_msg.lower() or "api_key" in error_msg.lower():
                return "# ERROR: OpenAI API key invalid. Please check OPENAI_API_KEY environment variable."
            elif "rate_limit" in error_msg.lower():
                return "# ERROR: OpenAI API rate limit exceeded. Please try again later."
            else:
                return f"# ERROR: {type(e).__name__}: {str(e)}"

    def reset_conversation(self):
        """대화 기록 초기화"""
        self.conversation_history = []


# 간단한 테스트
if __name__ == "__main__":
    import sys

    agent = LLMRobotAgent()

    if len(sys.argv) > 1:
        command = ' '.join(sys.argv[1:])
    else:
        command = "정사각형 경로로 움직여줘, 크기는 1.5미터로"

    print(f"User Command: {command}")
    print("\nGenerated Code:")
    print("="*60)
    code = agent.generate_code(command)
    print(code)
    print("="*60)
