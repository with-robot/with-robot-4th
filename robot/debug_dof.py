"""Identify the joint corresponding to a specific DOF index"""
import mujoco
import sys

model_path = "../model/robocasa/panda_omron.xml"
target_dof = 104

try:
    print(f"Loading model from {model_path}...", flush=True)
    model = mujoco.MjModel.from_xml_path(model_path)
    
    print(f"Total DOFs: {model.nv}")
    
    if target_dof >= model.nv:
        print(f"Error: Target DOF {target_dof} is out of range (0-{model.nv-1})")
        sys.exit(1)

    # Iterate through joints to find which one contains the target DOF
    # model.jnt_dofadr[j] gives the starting DOF address of joint j
    found = False
    for j in range(model.njnt):
        dof_start = model.jnt_dofadr[j]
        # Determine DOF end (depends on joint type, usually 1, 3, or 6)
        # But we can just check if target_dof is >= start and < next_start
        
        # Simple way: check if target_dof matches
        # A joint can have multiple DOFs (e.g. free joint has 6)
        
        # Let's find the joint name
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        
        # Check if this joint includes the target DOF
        # We need to know how many DOFs this joint has.
        # We can infer it by looking at the next joint's address or total nv
        
        # Actually, let's just print all joints and their DOF ranges to be safe
        pass

    print("\nSearching for DOF 104...")
    for j in range(model.njnt):
        dof_adr = model.jnt_dofadr[j]
        # How many DOFs?
        # qpos address is jnt_qposadr[j]
        # We can look at the type
        jnt_type = model.jnt_type[j]
        
        dof_count = 0
        if jnt_type == mujoco.mjtJoint.mjJNT_FREE:
            dof_count = 6
        elif jnt_type == mujoco.mjtJoint.mjJNT_BALL:
            dof_count = 3
        elif jnt_type == mujoco.mjtJoint.mjJNT_SLIDE:
            dof_count = 1
        elif jnt_type == mujoco.mjtJoint.mjJNT_HINGE:
            dof_count = 1
            
        if dof_adr <= target_dof < dof_adr + dof_count:
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
            print(f"FOUND! DOF {target_dof} belongs to Joint: '{name}' (ID: {j})")
            print(f"  Joint Type: {jnt_type}")
            print(f"  DOF Range: {dof_adr} - {dof_adr + dof_count - 1}")
            found = True
            break
            
    if not found:
        print("Could not find joint for DOF 104. It might be part of a composite body or I missed something.")

except Exception as e:
    print(f"Error: {e}")
