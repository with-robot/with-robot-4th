#!/usr/bin/env python3
"""
초간단 로봇 제어 프로그램
명령어 입력만으로 로봇을 제어합니다
"""

import requests
from math import pi as PI

API_URL = "http://localhost:8800/send_action"


def send(code):
    """명령 전송"""
    try:
        response = requests.post(API_URL,
            json={'action': {'type': 'run_code', 'payload': {'code': code}}},
            timeout=2)
        if response.status_code == 200:
            print("✅ 명령 전송 완료!")
            return True
        else:
            print("❌ 전송 실패")
            return False
    except Exception as e:
        print(f"❌ 오류: {e}")
        return False


def move(x, y, theta=0):
    """간단 이동 함수"""
    print(f"🤖 이동: ({x}, {y}, {theta:.2f})")
    code = f"set_target_position({x}, {y}, {theta}, wait=True); print('도착!')"
    send(code)


print("\n" + "="*60)
print("🤖 간단 로봇 제어")
print("="*60)
print("\n명령어:")
print("  숫자 입력: 'x y theta' (예: 1 0.5 1.57)")
print("  빠른 명령:")
print("    f  - 앞으로 (0.5m)")
print("    b  - 뒤로 (0.5m)")
print("    l  - 왼쪽 회전 (90도)")
print("    r  - 오른쪽 회전 (90도)")
print("    h  - 원점 복귀")
print("    s  - 사각형")
print("    c  - 원형")
print("    q  - 종료")
print("="*60 + "\n")

# 현재 위치 추적
x, y, theta = 0.0, 0.0, 0.0

while True:
    try:
        cmd = input("🎮 명령 입력> ").strip().lower()

        if not cmd:
            continue

        # 종료
        if cmd == 'q':
            print("👋 종료")
            break

        # 빠른 명령
        elif cmd == 'f':  # 앞으로
            import math
            x += 0.5 * math.cos(theta)
            y += 0.5 * math.sin(theta)
            move(x, y, theta)

        elif cmd == 'b':  # 뒤로
            import math
            x -= 0.5 * math.cos(theta)
            y -= 0.5 * math.sin(theta)
            move(x, y, theta)

        elif cmd == 'l':  # 좌회전
            theta += PI/2
            move(x, y, theta)

        elif cmd == 'r':  # 우회전
            theta -= PI/2
            move(x, y, theta)

        elif cmd == 'h':  # 원점
            x, y, theta = 0, 0, 0
            move(x, y, theta)

        elif cmd == 's':  # 사각형
            print("🔲 사각형 시작")
            code = """
set_target_position(1, 0, 0, wait=True)
set_target_position(1, 1, 1.57, wait=True)
set_target_position(0, 1, 3.14, wait=True)
set_target_position(0, 0, -1.57, wait=True)
print('사각형 완료!')
"""
            send(code)
            x, y, theta = 0, 0, -1.57

        elif cmd == 'c':  # 원형
            print("⭕ 원형 시작")
            code = """
import math
for i in range(17):
    angle = 2 * PI * i / 16
    x = 0.5 * (1 - math.cos(angle))
    y = 0.5 * math.sin(angle)
    set_target_position(x, y, angle, wait=True)
print('원형 완료!')
"""
            send(code)
            x, y, theta = 0, 0, 0

        # 좌표 입력 (x y theta)
        else:
            try:
                parts = cmd.split()
                if len(parts) >= 2:
                    x = float(parts[0])
                    y = float(parts[1])
                    theta = float(parts[2]) if len(parts) > 2 else 0
                    move(x, y, theta)
                else:
                    print("❌ 형식: x y [theta] (예: 1 0.5 1.57)")
            except ValueError:
                print("❌ 숫자를 입력하세요")

    except KeyboardInterrupt:
        print("\n\n👋 Ctrl+C로 종료")
        break
    except Exception as e:
        print(f"❌ 오류: {e}")

print("\n✅ 프로그램 종료")
