"""Test wider range of movement to find workable coordinates"""
import sys
sys.path.append('.')

from simulator import MujocoSimulator
import code_repository
import time

# Initialize
sim = MujocoSimulator(xml_path="../model/robocasa/panda_omron.xml")
code_repository.simulator = sim

print("Testing robot movement in wider range...")
print("Starting at:", sim.get_current_position())

# Test simple movements
test_targets = [
    ("Forward (Y+)", 0.0, 0.5, 0.0),
    ("Right (X+)", 0.5, 0.0, 0.0),
    ("Diagonal", 0.5, 0.5, 0.0),
]

for name, x, y, theta in test_targets:
    print(f"\n{'='*60}")
    print(f"Test: {name} -> ({x}, {y}, {theta})")
    print("="*60)
    
    sim.set_target_position(x, y, theta)
    
    # Wait and check position
    time.sleep(5)
    current = sim.get_current_position()
    error = sim.get_position_diff()
    
    print(f"Current: ({current[0]:.3f}, {current[1]:.3f}, {current[2]:.3f})")
    print(f"Error: ({error[0]:.3f}, {error[1]:.3f}, {error[2]:.3f})")
    print(f"Error magnitude: {(error[0]**2 + error[1]**2)**0.5:.3f}")

print("\n" + "="*60)
print("Test complete - check MuJoCo viewer to see robot movement")
print("="*60)
