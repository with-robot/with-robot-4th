"""Quick debug to verify coordinates and robot movement"""
import sys
sys.path.append('.')
import code_repository

print("="*60)
print("COORDINATE DEBUG")
print("="*60)

# Get coordinates
start = code_repository.get_safe_start_point()
target = code_repository.get_safe_target()

print(f"\nSafe Start Point (Relative):")
print(f"  {start}")
print(f"  Global: ({start[0] + 0.8:.2f}, {start[1] + (-3.45):.2f})")

print(f"\nSafe Target (Relative):")
print(f"  {target}")
print(f"  Global: ({target[0] + 0.8:.2f}, {target[1] + (-3.45):.2f})")

mid_x = (start[0] + target[0]) / 2
mid_y = (start[1] + target[1]) / 2
print(f"\nMidpoint for Obstacle (Relative):")
print(f"  ({mid_x:.2f}, {mid_y:.2f})")
print(f"  Global: ({mid_x + 0.8:.2f}, {mid_y + (-3.45):.2f})")

print(f"\nTotal Movement Distance: {((target[0]-start[0])**2 + (target[1]-start[1])**2)**0.5:.2f}m")
print("="*60)
