"""Test if robot can actually move in the running simulation"""
import requests
import time

print("="*60)
print("DIRECT API TEST - Send movement command to running server")
print("="*60)

# Test simple movement via API
test_code = """
print("TEST: Moving robot to (2, 0, 0)")
set_target_position(2, 0, 0, wait=True)
print("TEST: Movement complete")
"""

payload = {
    "action": {
        "type": "run_code",
        "payload": {
            "code": test_code
        }
    }
}

print("\nSending command to http://localhost:8800/send_action...")
try:
    response = requests.post("http://localhost:8800/send_action", json=payload, timeout=30)
    print(f"Response status: {response.status_code}")
    print(f"Response: {response.json()}")
except Exception as e:
    print(f"Error: {e}")

print("\nCheck MuJoCo viewer - did the robot move 2 meters in X direction?")
print("="*60)
