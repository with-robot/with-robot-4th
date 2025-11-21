"""Static XML analysis to find DOF 104"""
import xml.etree.ElementTree as ET
import sys

model_path = "../model/robocasa/panda_omron.xml"

try:
    print(f"Parsing {model_path}...")
    tree = ET.parse(model_path)
    root = tree.getroot()
    
    dof_count = 0
    target_dof = 104
    
    # Helper to find joints recursively or just iterate all elements if flattened?
    # MuJoCo XMLs are hierarchical. We need to traverse in order.
    # Actually, the order of DOFs in MuJoCo corresponds to the order of joints in the XML *depth-first* traversal?
    # Or just document order? Usually document order of appearance.
    
    print("Traversing XML to count DOFs...")
    
    found = False
    
    # We need to traverse the tree and look for 'joint' tags
    # The order matters.
    
    for elem in root.iter():
        if elem.tag == 'joint':
            j_type = elem.get('type', 'hinge') # default is hinge
            name = elem.get('name', 'unnamed')
            
            current_dofs = 0
            if j_type == 'free':
                current_dofs = 6
            elif j_type == 'ball':
                current_dofs = 3
            elif j_type == 'slide':
                current_dofs = 1
            elif j_type == 'hinge':
                current_dofs = 1
            else:
                current_dofs = 0 # unknown
                
            start_dof = dof_count
            end_dof = dof_count + current_dofs - 1
            
            dof_count += current_dofs
            
            if start_dof <= target_dof <= end_dof:
                print(f"\nFOUND! DOF {target_dof} is in joint: '{name}'")
                print(f"  Type: {j_type}")
                print(f"  DOF Range: {start_dof} - {end_dof}")
                
                # Print parent body if possible (hard with iter, but let's try to infer)
                # We can't easily get parent with iter.
                found = True
                break
                
    print(f"\nTotal DOFs counted: {dof_count}")

except Exception as e:
    print(f"Error: {e}")
