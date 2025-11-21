#!/usr/bin/env python3
"""Actuator 설정 디버깅 스크립트"""

import mujoco

# XML 로드
xml_path = "../model/robocasa/panda_omron.xml"
print("=" * 60)
print("🔍 MuJoCo Actuator 디버깅")
print("=" * 60)

try:
    model = mujoco.MjModel.from_xml_path(xml_path)
    print(f"\n✅ XML 로드 성공: {xml_path}")
except Exception as e:
    print(f"\n❌ XML 로드 실패: {e}")
    exit(1)

# Joint 정보
print("\n📍 Mobile Base Joints:")
joint_names = [
    "mobilebase0_joint_mobile_side",
    "mobilebase0_joint_mobile_forward",
    "mobilebase0_joint_mobile_yaw"
]

for name in joint_names:
    try:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        print(f"   ✓ {name:40s} -> ID: {joint_id}")
    except Exception as e:
        print(f"   ✗ {name:40s} -> ERROR: {e}")

# Actuator 정보
print("\n⚙️  Mobile Base Actuators:")
actuator_names = [
    "mobilebase0_actuator_mobile_side",
    "mobilebase0_actuator_mobile_forward",
    "mobilebase0_actuator_mobile_yaw"
]

for name in actuator_names:
    try:
        act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        # Actuator 설정 확인
        ctrl_range = model.actuator_ctrlrange[act_id]
        force_range = model.actuator_forcerange[act_id]
        print(f"   ✓ {name:40s} -> ID: {act_id}")
        print(f"      Ctrl range: [{ctrl_range[0]:.2f}, {ctrl_range[1]:.2f}]")
        print(f"      Force range: [{force_range[0]:.1f}, {force_range[1]:.1f}]")
    except Exception as e:
        print(f"   ✗ {name:40s} -> ERROR: {e}")

# 시뮬레이션 데이터 생성 및 PD 컨트롤 테스트
print("\n🧪 PD Controller 테스트:")
data = mujoco.MjData(model)

# Joint IDs
joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
             for name in joint_names]
actuator_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                for name in actuator_names]

print(f"   Joint IDs: {joint_ids}")
print(f"   Actuator IDs: {actuator_ids}")

# 초기 위치
import numpy as np
print(f"\n   초기 위치: x={data.qpos[joint_ids[0]]:.3f}, y={data.qpos[joint_ids[1]]:.3f}, theta={data.qpos[joint_ids[2]]:.3f}")

# 목표 설정 및 PD 계산
target = np.array([1.0, 0.5, 0.0])
KP = np.array([2.0, 2.0, 1.5])
KD = np.array([0.5, 0.5, 0.3])

current_pos = np.array([
    data.qpos[joint_ids[0]],
    data.qpos[joint_ids[1]],
    data.qpos[joint_ids[2]]
])
current_vel = np.array([
    data.qvel[joint_ids[0]],
    data.qvel[joint_ids[1]],
    data.qvel[joint_ids[2]]
])

pos_error = target - current_pos
control = KP * pos_error - KD * current_vel

print(f"   목표 위치: x={target[0]:.3f}, y={target[1]:.3f}, theta={target[2]:.3f}")
print(f"   위치 오차: {pos_error}")
print(f"   계산된 제어: {control}")
print(f"   제어 범위 체크:")
print(f"      X (side): {control[0]:.3f} in [-1.0, 1.0]? {-1.0 <= control[0] <= 1.0}")
print(f"      Y (forward): {control[1]:.3f} in [-1.0, 1.0]? {-1.0 <= control[1] <= 1.0}")
print(f"      Theta (yaw): {control[2]:.3f} in [-1.5, 1.5]? {-1.5 <= control[2] <= 1.5}")

print("\n" + "=" * 60)
print("✅ 디버깅 완료!")
print("=" * 60)
