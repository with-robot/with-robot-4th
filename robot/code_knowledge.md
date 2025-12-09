# Mobile Manipulator Robot Control API

## Environment & Constraints
- **Sandbox**: Restricted Python. **import not allowed**. `time` and `math` are pre-loaded.
- **Coordinates**: World frame (meters, radians).
- **Mobile Base**: x, y, theta.
- **Arm**: 7-DOF Panda.
- **EE**: Position [x, y, z] only.

## Mobile Base Control

`get_mobile_position()` -> `list[float]`
Returns current base position `[x, y, theta]`.

`set_mobile_target_position(mobile_target_position, timeout=10.0, verbose=False)` -> `bool`
Sets target `[x, y, theta]`. Returns `True` if converged.

`plan_mobile_path(target_joint, grid_size=0.1)` -> `list[list[float]] | None`
Plans A* path to `[x, y]` (world coords). Returns list of `[x, y, theta]` waypoints or `None`.

`get_grid_map()` -> `list[list[int]]`
Returns 2D binary occupancy grid (0=free, 1=occupied). 0.1m cell size.

`follow_mobile_path(path_world, timeout_per_waypoint=30.0, verbose=False)` -> `bool`
Follows path from `plan_mobile_path`.

## Arm Control

`get_arm_joint_position()` -> `list[float]`
Returns 7 joint angles `[j1..j7]` in radians.

`set_arm_target_joint(arm_target_position, timeout=10.0, verbose=False)` -> `bool`
Sets 7 joint angles. Returns `True` if converged.

## End Effector & Gripper

`get_ee_position()` -> `tuple(list[float], list[float])`
Returns `([x,y,z], [roll,pitch,yaw])` in world frame.

`set_ee_target_position(target_pos, timeout=10.0, verbose=False)` -> `bool`
Sets EE position `[x, y, z]`. Orientation is not controlled.

`get_gripper_width()` -> `float`
Returns width in meters.

`set_target_gripper_width(target_width, timeout=10.0, verbose=False)` -> `bool`
Sets width (0.0=closed, 0.08=open).

## High-Level Operations

`pick_object(object_pos, approach_height=0.1, lift_height=0.2, return_to_home=True, timeout=10.0, verbose=False)` -> `bool`
Executes pick sequence at `object_pos`.

`place_object(place_pos, approach_height=0.2, retract_height=0.3, return_to_home=True, timeout=10.0, verbose=False)` -> `bool`
Executes place sequence at `place_pos`.

`get_object_positions()` -> `dict`
Returns dict of objects: `{'name': {'id': int, 'pos': [x,y,z], 'ori': [r,p,y]}}`.

## Constants
- `PI`: 3.14159...
- `RESULT`: Dict for return values.

## Examples

### Pick & Place Workflow
```python
objects = get_object_positions()
# object keys may vary. check available keys first if needed.
object_pos = objects['object_red_0']['pos']
bowl_pos = objects['object_yellow_bowl_7']['pos']

# 1. Approach and Pick
path = plan_mobile_path(object_pos)
if path:
    follow_mobile_path(path)
pick_success = pick_object(object_pos, approach_height=0.1, lift_height=0.2)

if not pick_success:
    RESULT['status'] = 'pick_failed'
else:
    # 2. Move to Place Location
    path = plan_mobile_path(bowl_pos)
    if path:
        follow_mobile_path(path)
    # 3. Place Object
    place_success = place_object(bowl_pos, approach_height=0.2, retract_height=0.3)
    RESULT['status'] = 'completed' if place_success else 'place_failed'
```