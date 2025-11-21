"""Direct test of Scenario 1 obstacle avoidance"""
import sys
import os
sys.path.append(os.path.dirname(__file__))

from simulator import MujocoSimulator
import code_repository

# Initialize
sim = MujocoSimulator(xml_path="../model/robocasa/panda_omron.xml")
code_repository.simulator = sim

print("=" * 60)
print("Testing Scenario 1: Obstacle Avoidance")
print("=" * 60)

# Execute scenario
test_code = """
print("\\n1. Getting current position...")
current_pos = get_current_position()
print(f"   Current: {current_pos}")

print("\\n2. Moving to safe start point...")
start_x, start_y, start_theta = get_safe_start_point()
print(f"   Target: ({start_x}, {start_y}, {start_theta})")
set_target_position(start_x, start_y, start_theta, wait=True)
print("   ✓ Reached start point")

print("\\n3. Getting safe target...")
target_x, target_y, target_theta = get_safe_target()
print(f"   Target: ({target_x}, {target_y}, {target_theta})")

print("\\n4. Placing obstacle at midpoint...")
mid_x = (start_x + target_x) / 2
mid_y = (start_y + target_y) / 2
print(f"   Obstacle at: ({mid_x:.2f}, {mid_y:.2f})")
add_obstacle(mid_x, mid_y, radius=0.3)

print("\\n5. Moving to target (will replan if obstacle detected)...")
set_target_position(target_x, target_y, target_theta, wait=True)
print("   ✓ Reached target!")

print("\\n✅ Scenario 1 Complete!")
"""

try:
    code_repository.exec_code(test_code)
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
