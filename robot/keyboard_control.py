#!/usr/bin/env python3
"""
실시간 키보드 제어 프로그램
방향키로 로봇을 수동 제어합니다

사용법: python3 keyboard_control.py
"""

import sys
import tty
import termios
import requests
from math import pi as PI

API_URL = "http://localhost:8800/send_action"

# 제어 파라미터
MOVE_STEP = 0.1      # 이동 거리 (미터)
ROTATE_STEP = PI/8   # 회전 각도 (22.5도)

# 현재 로봇 위치 (추정치)
current_x = 0.0
current_y = 0.0
current_theta = 0.0


def send_move(x, y, theta):
    """로봇 이동 명령 전송 (비동기)"""
    global current_x, current_y, current_theta
    current_x = x
    current_y = y
    current_theta = theta

    code = f"set_target_position({x}, {y}, {theta}, wait=False)"
    try:
        response = requests.post(API_URL,
            json={'action': {'type': 'run_code', 'payload': {'code': code}}},
            timeout=0.5)
        return response.status_code == 200
    except:
        return False


def get_key():
    """키 입력 받기 (non-blocking)"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        # 방향키는 3글자 시퀀스
        if ch == '\x1b':
            ch += sys.stdin.read(2)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def print_status():
    """현재 상태 출력"""
    print(f"\r로봇 위치: X={current_x:+.2f}m, Y={current_y:+.2f}m, θ={current_theta:.2f}rad   ", end='', flush=True)


def main():
    """메인 제어 루프"""
    global current_x, current_y, current_theta

    print("="*60)
    print("🎮 실시간 키보드 제어 모드")
    print("="*60)
    print("\n조작 방법:")
    print("  ↑ : 앞으로 이동")
    print("  ↓ : 뒤로 이동")
    print("  ← : 왼쪽으로 회전")
    print("  → : 오른쪽으로 회전")
    print("  w : Y+ 방향 이동")
    print("  s : Y- 방향 이동")
    print("  a : X- 방향 이동")
    print("  d : X+ 방향 이동")
    print("  h : 원점으로 복귀 (Home)")
    print("  q : 종료")
    print("\n이동 단위: {:.2f}m, 회전 단위: {:.1f}도".format(MOVE_STEP, ROTATE_STEP * 180 / PI))
    print("="*60)
    print("\n준비! 키를 눌러 제어하세요...")

    try:
        while True:
            print_status()
            key = get_key()

            # 종료
            if key == 'q' or key == 'Q':
                print("\n\n👋 종료합니다")
                break

            # 원점 복귀
            elif key == 'h' or key == 'H':
                print("\n🏠 원점으로 복귀")
                current_x, current_y, current_theta = 0, 0, 0
                send_move(current_x, current_y, current_theta)

            # 방향키 - 전진/후진
            elif key == '\x1b[A':  # ↑
                dx = MOVE_STEP * __import__('math').cos(current_theta)
                dy = MOVE_STEP * __import__('math').sin(current_theta)
                current_x += dx
                current_y += dy
                send_move(current_x, current_y, current_theta)
                print("\n↑ 전진")

            elif key == '\x1b[B':  # ↓
                dx = MOVE_STEP * __import__('math').cos(current_theta)
                dy = MOVE_STEP * __import__('math').sin(current_theta)
                current_x -= dx
                current_y -= dy
                send_move(current_x, current_y, current_theta)
                print("\n↓ 후진")

            # 방향키 - 회전
            elif key == '\x1b[D':  # ←
                current_theta += ROTATE_STEP
                send_move(current_x, current_y, current_theta)
                print("\n← 좌회전")

            elif key == '\x1b[C':  # →
                current_theta -= ROTATE_STEP
                send_move(current_x, current_y, current_theta)
                print("\n→ 우회전")

            # WASD - 절대 좌표 이동
            elif key == 'w' or key == 'W':
                current_y += MOVE_STEP
                send_move(current_x, current_y, current_theta)
                print("\nW Y+ 이동")

            elif key == 's' or key == 'S':
                current_y -= MOVE_STEP
                send_move(current_x, current_y, current_theta)
                print("\nS Y- 이동")

            elif key == 'a' or key == 'A':
                current_x -= MOVE_STEP
                send_move(current_x, current_y, current_theta)
                print("\nA X- 이동")

            elif key == 'd' or key == 'D':
                current_x += MOVE_STEP
                send_move(current_x, current_y, current_theta)
                print("\nD X+ 이동")

    except KeyboardInterrupt:
        print("\n\n⚠️  Ctrl+C로 중단됨")
    except Exception as e:
        print(f"\n\n❌ 오류: {e}")

    # 종료 시 원점 복귀
    print("\n원점으로 복귀 중...")
    send_move(0, 0, 0)
    print("✅ 완료")


if __name__ == "__main__":
    # 서버 연결 확인
    try:
        response = requests.get("http://localhost:8800/", timeout=2)
        if response.status_code != 200:
            print("❌ 서버가 응답하지 않습니다. 시뮬레이터를 먼저 실행하세요.")
            sys.exit(1)
    except:
        print("❌ 서버에 연결할 수 없습니다.")
        print("   먼저 시뮬레이터를 실행하세요:")
        print("   mjpython robot/main.py")
        sys.exit(1)

    main()
