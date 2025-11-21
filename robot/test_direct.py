#!/usr/bin/env python3
"""
Direct test - 시뮬레이터 없이 코드만 테스트
"""

# 간단한 모의 시뮬레이터
class MockSimulator:
    def __init__(self):
        self.target = [0, 0, 0]

    def set_target_position(self, x, y, theta):
        self.target = [x, y, theta]
        print(f"✅ Target set: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")

    def get_target_position(self):
        return self.target

    def get_position_diff(self):
        return [0.05, 0.05, 0.05]  # 작은 오차

# code_repository 모의
class MockCodeRepository:
    def __init__(self):
        self.simulator = MockSimulator()

import sys
import numpy as np

# Mock 설정
mock_repo = MockCodeRepository()

def set_target_position(x, y, theta, wait=True):
    """모의 set_target_position"""
    mock_repo.simulator.set_target_position(x, y, theta)
    if wait:
        print(f"   → Waiting for robot to reach target...")

# 생성된 코드 테스트
print("="*60)
print("🧪 Testing Generated Code (without MuJoCo)")
print("="*60)

PI = np.pi

# 웹에서 생성된 코드 실행
print("\n[Running: 원형으로 천천히 돌아줘]\n")

import math
print("Starting slow circular motion...")
radius = 0.5
steps = 32  # Increased steps for slower motion
for i in range(steps + 1):
    angle = 2 * PI * i / steps
    x = radius * (1 - math.cos(angle))
    y = radius * math.sin(angle)
    theta = angle
    set_target_position(x, y, theta, wait=True)
    if i % 8 == 0:  # Adjusted for increased steps
        print(f"Progress: {i*100//steps}%")
print("Slow circle motion complete!")

print("\n" + "="*60)
print("✅ Code execution test completed!")
print("="*60)
