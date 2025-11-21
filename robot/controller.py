#!/usr/bin/env python3
"""
MuJoCo 로봇 간편 컨트롤러
사용법: python controller.py
"""

import requests
import json
from math import pi as PI

# 서버 주소
API_URL = "http://localhost:8800/send_action"


def send_code(code):
    """Python 코드를 로봇에게 전송"""
    response = requests.post(API_URL,
        json={
            'action': {
                'type': 'run_code',
                'payload': {'code': code}
            }
        })
    return response.json()


def move_to(x, y, theta=0, wait=True):
    """로봇을 특정 위치로 이동"""
    code = f"set_target_position({x}, {y}, {theta}, wait={wait})"
    print(f"🤖 로봇 이동: x={x}, y={y}, theta={theta:.2f}")
    return send_code(code)


def go_home():
    """로봇을 원점으로 이동"""
    print("🏠 원점으로 복귀")
    return move_to(0, 0, 0)


def square_path(size=1.0):
    """사각형 경로로 이동"""
    print(f"🔲 사각형 경로 시작 (크기: {size}m)")
    code = f"""
print("사각형 경로 시작...")
set_target_position({size}, 0, 0, wait=True)
print("1/4 완료")
set_target_position({size}, {size}, {PI/2}, wait=True)
print("2/4 완료")
set_target_position(0, {size}, {PI}, wait=True)
print("3/4 완료")
set_target_position(0, 0, {-PI/2}, wait=True)
print("4/4 완료 - 사각형 완성!")
"""
    return send_code(code)


def circle_path(radius=0.5, steps=16):
    """원형 경로로 이동"""
    print(f"⭕ 원형 경로 시작 (반지름: {radius}m, {steps}단계)")
    code = f"""
import math
radius = {radius}
steps = {steps}
for i in range(steps + 1):
    angle = 2 * PI * i / steps
    x = radius * (1 - math.cos(angle))
    y = radius * math.sin(angle)
    theta = angle
    set_target_position(x, y, theta, wait=True)
    if i % 4 == 0:
        print(f"진행률: {{i*100//steps}}%")
print("원형 경로 완료!")
"""
    return send_code(code)


def rotate_in_place(rotations=1):
    """제자리에서 회전"""
    print(f"🔄 제자리 회전 {rotations}바퀴")
    code = f"""
for i in range({rotations * 8}):
    angle = 2 * PI * i / 8
    set_target_position(0, 0, angle, wait=True)
print("회전 완료!")
"""
    return send_code(code)


def custom_code(code):
    """사용자 정의 코드 실행"""
    print("💻 사용자 코드 실행")
    return send_code(code)


def main():
    """메인 인터랙티브 메뉴"""
    print("\n" + "="*60)
    print("🤖 MuJoCo 로봇 컨트롤러")
    print("="*60)

    while True:
        print("\n📋 메뉴:")
        print("  1. 특정 위치로 이동")
        print("  2. 원점으로 복귀")
        print("  3. 사각형 경로")
        print("  4. 원형 경로")
        print("  5. 제자리 회전")
        print("  6. 사용자 코드 실행")
        print("  0. 종료")

        choice = input("\n선택 (0-6): ").strip()

        if choice == '0':
            print("👋 종료합니다")
            break

        elif choice == '1':
            try:
                x = float(input("X 좌표 (미터): "))
                y = float(input("Y 좌표 (미터): "))
                theta = float(input("각도 (라디안, 기본값 0): ") or "0")
                result = move_to(x, y, theta)
                print(f"✅ 결과: {result}")
            except ValueError:
                print("❌ 잘못된 입력입니다")

        elif choice == '2':
            result = go_home()
            print(f"✅ 결과: {result}")

        elif choice == '3':
            size = float(input("사각형 크기 (미터, 기본값 1.0): ") or "1.0")
            result = square_path(size)
            print(f"✅ 결과: {result}")

        elif choice == '4':
            radius = float(input("반지름 (미터, 기본값 0.5): ") or "0.5")
            result = circle_path(radius)
            print(f"✅ 결과: {result}")

        elif choice == '5':
            rotations = int(input("회전 횟수 (기본값 1): ") or "1")
            result = rotate_in_place(rotations)
            print(f"✅ 결과: {result}")

        elif choice == '6':
            print("Python 코드를 입력하세요 (빈 줄로 종료):")
            lines = []
            while True:
                line = input()
                if not line:
                    break
                lines.append(line)
            code = '\n'.join(lines)
            result = custom_code(code)
            print(f"✅ 결과: {result}")

        else:
            print("❌ 잘못된 선택입니다")


if __name__ == "__main__":
    # 빠른 테스트 함수들
    # move_to(1.0, 0.5, PI/4)
    # square_path(1.0)
    # circle_path(0.5)
    # rotate_in_place(2)

    # 인터랙티브 메뉴 실행
    main()
