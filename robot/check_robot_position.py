import sys
import os
import time

# Add robot directory to path
sys.path.append(os.path.join(os.getcwd(), "robot"))

from simulator import MujocoSimulator

def check_position():
    print("Initializing Simulator to check robot state...")
    # Initialize simulator
    sim = MujocoSimulator(xml_path="../model/robocasa/panda_omron.xml")
    
    # Get current position
    pos = sim.get_current_position()
    print(f"\n📍 ROBOT INITIAL POSITION: {pos}")
    print(f"   X: {pos[0]:.4f}")
    print(f"   Y: {pos[1]:.4f}")
    print(f"   Theta: {pos[2]:.4f}")
    
    # Check limits/bounds if possible (manual inspection of printed values)
    print("\nChecking for potential collisions at start...")
    # We can't easily check collisions without stepping, but position is key.

if __name__ == "__main__":
    check_position()
