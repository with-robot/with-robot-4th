"""Basic simulator test - check if robot can move at all"""
import sys
sys.path.append('.')

from simulator import MujocoSimulator
import time
import numpy as np

print("="*60)
print("BASIC SIMULATOR TEST")
print("="*60)

# Initialize
sim = MujocoSimulator(xml_path="../model/robocasa/panda_omron.xml")

print("\n1. Initial state:")
print(f"   Position: {sim.get_current_position()}")
print(f"   Velocity: {sim.get_current_velocity()}")
print(f"   Target: {sim.get_target_position()}")

print("\n2. Setting target to (1.0, 0.0, 0.0)")
sim.set_target_position(1.0, 0.0, 0.0)
print(f"   New target: {sim.get_target_position()}")

print("\n3. Computing control signal...")
control = sim._compute_control()
print(f"   Control output: {control}")
print(f"   KP gains: {sim.model.actuator('mobilebase0_actuator_mobile_side')}")

print("\n4. Checking actuator settings...")
for i, name in enumerate(['mobile_side', 'mobile_forward', 'mobile_yaw']):
    act_id = sim.actuator_ids[i]
    print(f"   {name}:")
    print(f"     ctrl_range: {sim.model.actuator_ctrlrange[act_id]}")
    print(f"     force_range: {sim.model.actuator_forcerange[act_id]}")

print("\n5. Checking joint settings...")  
for i, name in enumerate(['mobile_side', 'mobile_forward', 'mobile_yaw']):
    jnt_id = sim.joint_ids[i]
    print(f"   {name}:")
    print(f"     qpos: {sim.data.qpos[jnt_id]}")
    print(f"     qvel: {sim.data.qvel[jnt_id]}")

print("\n" + "="*60)
print("If control output is near zero, the robot won't move!")
print("Expected control ~[0.8, 0, 0] for target (1, 0, 0)")
print("="*60)
