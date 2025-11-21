#!/usr/bin/env python3
"""간단한 시뮬레이터 테스트"""

import sys
sys.path.append('/Users/chloepark/Desktop/with-robot-4th/robot')

from simulator import MujocoSimulator
import numpy as np
import time

print("=" * 60)
print("🧪 시뮬레이터 테스트 (간단한 XML)")
print("=" * 60)

# 간단한 XML로 시뮬레이터 생성
sim = MujocoSimulator(xml_path="../model/robocasa/simple_robot.xml")
print("✅ 시뮬레이터 생성 완료")

# 초기 위치 확인
pos = sim.get_current_position()
print(f"초기 위치: x={pos[0]:.3f}, y={pos[1]:.3f}, theta={pos[2]:.3f}")

# 목표 설정
target = np.array([1.0, 0.5, 0.0])
sim.set_target_position(target[0], target[1], target[2])
print(f"목표 설정: x={target[0]:.3f}, y={target[1]:.3f}, theta={target[2]:.3f}")

# 수동으로 몇 스텝 실행
print("\n수동 시뮬레이션 (10 steps):")
import mujoco

for i in range(10):
    control = sim._compute_control()
    print(f"  Step {i}: control={control}, pos={sim.get_current_position()}")

    # Apply control
    sim.data.ctrl[sim.actuator_ids[0]] = control[0]
    sim.data.ctrl[sim.actuator_ids[1]] = control[1]
    sim.data.ctrl[sim.actuator_ids[2]] = control[2]

    # Step simulation
    mujoco.mj_step(sim.model, sim.data)
    time.sleep(0.01)

final_pos = sim.get_current_position()
print(f"\n최종 위치: x={final_pos[0]:.3f}, y={final_pos[1]:.3f}, theta={final_pos[2]:.3f}")
print(f"이동 거리: {np.linalg.norm(final_pos - pos):.3f}m")

print("\n" + "=" * 60)
print("✅ 테스트 완료!")
print("=" * 60)
