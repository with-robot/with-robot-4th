import mujoco
import numpy as np
import time
import sys

model_path = "../model/robocasa/panda_omron.xml"

def debug_physics():
    print(f"Loading model from {model_path}...", flush=True)
    try:
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"Failed to load model: {e}", flush=True)
        return

    print("\n=== 1. Verifying XML Changes ===", flush=True)
    
    # Check frictionloss
    joint_name = "mobilebase0_joint_mobile_forward"
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    
    if joint_id == -1:
        print(f"❌ Joint {joint_name} not found!", flush=True)
    else:
        friction = model.dof_frictionloss[model.jnt_dofadr[joint_id]]
        print(f"Joint '{joint_name}' frictionloss: {friction}", flush=True)
        if friction == 10:
            print("✅ Friction fix applied (10)", flush=True)
        else:
            print(f"❌ Friction fix NOT applied (Current: {friction}, Expected: 10)", flush=True)

    # Check actuator limits
    act_name = "mobilebase0_actuator_mobile_forward"
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
    
    if act_id == -1:
        print(f"❌ Actuator {act_name} not found!", flush=True)
    else:
        force_range = model.actuator_forcerange[act_id]
        ctrl_range = model.actuator_ctrlrange[act_id]
        print(f"Actuator '{act_name}' force range: {force_range}", flush=True)
        print(f"Actuator '{act_name}' ctrl range: {ctrl_range}", flush=True)
        
        if force_range[1] == 6000:
            print("✅ Force limit fix applied (6000)", flush=True)
        else:
            print(f"❌ Force limit fix NOT applied (Current: {force_range[1]}, Expected: 6000)", flush=True)

    print("\n=== 2. Testing Direct Body Movement (Bypassing Actuators) ===", flush=True)
    # Reset
    mujoco.mj_resetData(model, data)
    
    # Apply direct force to the mobile base body
    body_name = "mobilebase0_base"
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    
    print(f"Applying 100N force to '{body_name}' (ID: {body_id}) for 100 steps...", flush=True)
    
    initial_pos = data.qpos[model.jnt_qposadr[joint_id]]
    print(f"Initial Pos (Y): {initial_pos}", flush=True)
    
    for i in range(100):
        # Apply force in Y direction (forward)
        # xfrc_applied is 6D: [force_x, force_y, force_z, torque_x, torque_y, torque_z]
        data.xfrc_applied[body_id] = [0, 100, 0, 0, 0, 0]
        mujoco.mj_step(model, data)
        
    final_pos = data.qpos[model.jnt_qposadr[joint_id]]
    print(f"Final Pos (Y): {final_pos}", flush=True)
    print(f"Displacement: {final_pos - initial_pos}", flush=True)
    
    if abs(final_pos - initial_pos) > 0.01:
        print("✅ Robot MOVED with direct force. Kinematics are OK.", flush=True)
    else:
        print("❌ Robot did NOT move with direct force. It is physically stuck/anchored.", flush=True)

    print("\n=== 3. Testing Actuator Movement ===", flush=True)
    mujoco.mj_resetData(model, data)
    
    print(f"Applying max control {ctrl_range[1]} to actuator '{act_name}'...", flush=True)
    
    initial_pos = data.qpos[model.jnt_qposadr[joint_id]]
    
    for i in range(500): # 1 second at 2ms timestep
        data.ctrl[act_id] = ctrl_range[1]
        mujoco.mj_step(model, data)
        
    final_pos = data.qpos[model.jnt_qposadr[joint_id]]
    print(f"Final Pos (Y): {final_pos}", flush=True)
    print(f"Displacement: {final_pos - initial_pos}", flush=True)
    
    if abs(final_pos - initial_pos) > 0.01:
         print("✅ Robot MOVED with actuator.", flush=True)
    else:
         print("❌ Robot did NOT move with actuator.", flush=True)

if __name__ == "__main__":
    debug_physics()
