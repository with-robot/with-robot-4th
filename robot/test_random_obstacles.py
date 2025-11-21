import sys
import os

# Add robot directory to path
sys.path.append(os.path.join(os.getcwd(), "robot"))

from simulator import MujocoSimulator

def test_random_obstacles():
    print("Initializing Simulator...")
    # We don't need the viewer for this test
    sim = MujocoSimulator(xml_path="../model/robocasa/panda_omron.xml")
    
    print("Testing add_physical_obstacle...")
    added_obstacles = []
    
    for i in range(5):
        obs_id = sim.add_physical_obstacle(1.0 + i*0.5, 0.0)
        if obs_id:
            added_obstacles.append(obs_id)
            print(f"Added obstacle: {obs_id}")
        else:
            print("Failed to add obstacle")
            
    print("\nAdded Obstacles:", added_obstacles)
    
    # Check if we got any asset obstacles
    asset_obstacles = [obs for obs in added_obstacles if obs in ['apple', 'banana', 'bottled_water', 'can', 'mug']]
    print(f"Asset obstacles count: {len(asset_obstacles)}")
    
    if len(asset_obstacles) > 0:
        print("SUCCESS: Random asset obstacles were selected.")
    else:
        print("WARNING: No asset obstacles selected (could be bad luck or bug).")

if __name__ == "__main__":
    test_random_obstacles()
