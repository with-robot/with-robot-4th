#!/usr/bin/env python3
"""
Check action queue status and simulator state
"""

import requests
import json

API_URL = "http://localhost:8800"

# Test 1: Server alive?
try:
    response = requests.get(f"{API_URL}/")
    print("✅ Server is running")
    print(f"   {response.json()}")
except Exception as e:
    print(f"❌ Server error: {e}")
    exit(1)

# Test 2: Send simple command
print("\n" + "="*60)
print("Testing simple command...")
print("="*60)

test_code = """
print("TEST: Simple command")
set_target_position(0.5, 0, 0, wait=True)
print("TEST: Command completed")
"""

try:
    response = requests.post(
        f"{API_URL}/send_action",
        json={
            "action": {
                "type": "run_code",
                "payload": {"code": test_code}
            }
        }
    )
    print(f"✅ Command sent: {response.json()}")
    print("\nCheck terminal for execution logs...")
except Exception as e:
    print(f"❌ Error sending command: {e}")
