"""Direct test with larger coordinates and wait=False"""
import sys
sys.path.append('.')

from simulator import MujocoSimulator
import code_repository
import time

print("="*60)
print("DIRECT MOVEMENT TEST - No Wait")
print("="*60)

# Initialize simulator
sim = MujocoSimulator(xml_path="../model/robocasa/panda_omron.xml")
code_repository.simulator = sim

print("\n1. Initial position:", sim.get_current_position())

# Test with wait=False and larger movements
print("\n2. Setting target to (2.0, 2.0, 0.0) with wait=False")
sim.set_target_position(2.0, 2.0, 0.0)

# Check position over time
for i in range(10):
    time.sleep(1)
    pos = sim.get_current_position()
    target = sim.get_target_position()
    error = sim.get_position_diff()
    print(f"   t={i+1}s: pos=({pos[0]:.2f}, {pos[1]:.2f}), error={error[0]**2 + error[1]**2:.2f}")
    
print("\n3. Final position:", sim.get_current_position())
print("\nCheck MuJoCo viewer - did the robot move?")
print("="*60)

# Keep viewer open
input("\nPress Enter to close...")
