#!/usr/bin/env python3
"""
빠른 예제 모음집
각 함수를 실행하여 로봇 동작을 테스트할 수 있습니다
"""

import requests
from math import pi as PI

API_URL = "http://localhost:8800/send_action"


def send(code):
    """코드 전송 헬퍼 함수"""
    response = requests.post(API_URL, json={
        'action': {'type': 'run_code', 'payload': {'code': code}}
    })
    print(f"✅ 전송 완료: {response.json()}")


def example_1_basic_move():
    """예제 1: 기본 이동"""
    print("\n🎯 예제 1: 기본 이동")
    code = """
print("로봇 이동 시작")
set_target_position(1.0, 0, 0)
"""
    send(code)


def example_2_square():
    """예제 2: 사각형 그리기"""
    print("\n🔲 예제 2: 사각형 경로")
    code = """
print("사각형 경로 시작")
set_target_position(1.0, 0, 0, wait=True)
set_target_position(1.0, 1.0, PI/2, wait=True)
set_target_position(0, 1.0, PI, wait=True)
set_target_position(0, 0, -PI/2, wait=True)
print("사각형 완료!")
"""
    send(code)


def example_3_rotation():
    """예제 3: 제자리 회전"""
    print("\n🔄 예제 3: 제자리 회전")
    code = """
print("360도 회전 시작")
for i in range(9):
    angle = 2 * PI * i / 8
    set_target_position(0, 0, angle, wait=True)
    print(f"진행: {i}/8")
print("회전 완료!")
"""
    send(code)


def example_4_zigzag():
    """예제 4: 지그재그 이동"""
    print("\n⚡ 예제 4: 지그재그 패턴")
    code = """
print("지그재그 이동 시작")
positions = [
    (0.5, 0, 0),
    (0.5, 0.5, PI/4),
    (1.0, 0.5, 0),
    (1.0, 1.0, PI/4),
    (0, 0, PI)
]
for i, (x, y, theta) in enumerate(positions):
    set_target_position(x, y, theta, wait=True)
    print(f"지점 {i+1}/{len(positions)} 도착")
print("지그재그 완료!")
"""
    send(code)


def example_5_figure_eight():
    """예제 5: 8자 이동"""
    print("\n♾️  예제 5: 8자 패턴")
    code = """
import math
print("8자 패턴 시작")
steps = 32
for i in range(steps):
    t = 2 * PI * i / steps
    x = 0.5 * math.sin(t)
    y = 0.5 * math.sin(2 * t)
    theta = t
    set_target_position(x, y, theta, wait=True)
    if i % 8 == 0:
        print(f"진행률: {i*100//steps}%")
print("8자 패턴 완료!")
"""
    send(code)


def example_6_patrol():
    """예제 6: 순찰 경로"""
    print("\n🚓 예제 6: 순찰 경로")
    code = """
print("순찰 시작")
waypoints = [
    (1.0, 0, 0, "지점 A"),
    (1.0, 1.0, PI/2, "지점 B"),
    (0.5, 1.0, PI, "지점 C"),
    (0, 0.5, -PI/2, "지점 D"),
    (0, 0, 0, "원점")
]
for x, y, theta, name in waypoints:
    print(f"→ {name} 이동 중...")
    set_target_position(x, y, theta, wait=True)
    print(f"✓ {name} 도착")
print("순찰 완료!")
"""
    send(code)


def example_7_spiral():
    """예제 7: 나선형 이동"""
    print("\n🌀 예제 7: 나선형 패턴")
    code = """
import math
print("나선형 이동 시작")
steps = 24
for i in range(steps):
    t = 2 * PI * i / 8
    radius = i * 0.05
    x = radius * math.cos(t)
    y = radius * math.sin(t)
    theta = t
    set_target_position(x, y, theta, wait=True)
    if i % 6 == 0:
        print(f"진행: {i}/{steps}")
print("나선형 완료!")
"""
    send(code)


def main_menu():
    """인터랙티브 메뉴"""
    examples = {
        '1': example_1_basic_move,
        '2': example_2_square,
        '3': example_3_rotation,
        '4': example_4_zigzag,
        '5': example_5_figure_eight,
        '6': example_6_patrol,
        '7': example_7_spiral
    }

    print("\n" + "="*60)
    print("🤖 로봇 예제 모음")
    print("="*60)
    print("\n예제 목록:")
    print("  1. 기본 이동")
    print("  2. 사각형 그리기")
    print("  3. 제자리 회전")
    print("  4. 지그재그 이동")
    print("  5. 8자 패턴")
    print("  6. 순찰 경로")
    print("  7. 나선형 패턴")
    print("  0. 종료")

    while True:
        choice = input("\n실행할 예제 번호 (0-7): ").strip()

        if choice == '0':
            print("👋 종료")
            break
        elif choice in examples:
            examples[choice]()
        else:
            print("❌ 잘못된 선택입니다")


if __name__ == "__main__":
    # 특정 예제 직접 실행하려면 아래 주석 해제
    # example_1_basic_move()
    # example_2_square()
    # example_3_rotation()

    # 메뉴 실행
    main_menu()
