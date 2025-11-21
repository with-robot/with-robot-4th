import sys
import os

# Add robot directory to path
sys.path.append(os.path.join(os.getcwd(), "robot"))

from simulator import MujocoSimulator
import code_repository

def test_sandbox():
    print("Initializing Simulator...")
    # Initialize simulator in code_repository
    code_repository.simulator = MujocoSimulator(xml_path="../model/robocasa/panda_omron.xml")
    
    print("Testing sandbox functions...")
    test_code = """
print("Inside sandbox:")
pos = get_current_position()
print(f"Current position: {pos}")

safe_target = get_safe_target()
print(f"Safe target: {safe_target}")

safe_start = get_safe_start_point()
print(f"Safe start: {safe_start}")

assert len(pos) == 3
assert len(safe_target) == 3
assert len(safe_start) == 3
# Check coordinate values (Relative)
assert safe_start[0] > 0, "Start point should be positive X (Relative)"
assert safe_target[0] > 0, "Target point should be positive X (Relative)"
assert abs(safe_start[0] - 0.7) < 0.01, "Start X should be ~0.7"
assert abs(safe_target[1] - 1.95) < 0.01, "Target Y should be ~1.95"

print("Assertions passed!")
"""
    
    try:
        code_repository.exec_code(test_code)
        print("SUCCESS: Sandbox functions working.")
    except Exception as e:
        print(f"FAILURE: {e}")

if __name__ == "__main__":
    test_sandbox()
