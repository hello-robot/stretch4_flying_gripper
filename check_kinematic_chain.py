import coal # Do not remove this import, it helps pin import correctly on some systems
import pinocchio as pin
import yourdfpy
import warnings
import argparse
import difflib

STANDARD_CHAINS = {
    "Stretch Test Standard": [
        {"link": "virtual_base", "joint": "mobile_base_planar_joint", "type": "planar", "actuated": True, "dofs": 3},
        {"link": "base_link", "joint": "mast_joint", "type": "fixed", "actuated": False, "dofs": 0},
        {"link": "mast_link", "joint": "lift_joint", "type": "prismatic", "actuated": True, "dofs": 1},
        {"link": "lift_link", "joint": "arm_l0_joint", "type": "fixed", "actuated": False, "dofs": 0},
        {"link": "arm_l0_link", "joint": "arm_l4_joint", "type": "prismatic", "actuated": True, "dofs": 1},
        {"link": "arm_l4_link", "joint": "wrist_joint", "type": "fixed", "actuated": False, "dofs": 0},
        {"link": "wrist_link", "joint": "wrist_yaw_joint", "type": "revolute", "actuated": True, "dofs": 1},
        {"link": "wrist_yaw_link", "joint": "wrist_pitch_joint", "type": "revolute", "actuated": True, "dofs": 1},
        {"link": "wrist_pitch_link", "joint": "wrist_roll_joint", "type": "revolute", "actuated": True, "dofs": 1},
        {"link": "wrist_roll_link", "joint": "tool_attachment_site_joint", "type": "fixed", "actuated": False, "dofs": 0},
        {"link": "tool_attachment_site_link", "joint": "gripper_to_wrist_joint", "type": "fixed", "actuated": False, "dofs": 0},
        {"link": "quick_connect_interface_link", "joint": None, "type": None, "actuated": None, "dofs": None}
    ]
}

def main():
    parser = argparse.ArgumentParser(description="Print and verify kinematic chain of a URDF")
    parser.add_argument("urdf_path", help="Path to the URDF file")
    args = parser.parse_args()
    urdf_path = args.urdf_path

    check_kinematic_chain(urdf_path)
    
def check_kinematic_chain(urdf_path:str):
    target_link = "quick_connect_interface_link"
    
    # Load URDF via yourdfpy to easily trace the parent-child link tree
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        urdf = yourdfpy.URDF.load(urdf_path, load_meshes=False, load_collision_meshes=False)
        
    # Load via Pinocchio to get mathematically accurate DOF info
    model = pin.buildModelFromUrdf(urdf_path)
    
    # Build maps to traverse the tree backwards from the target
    parent_map = {}
    joint_by_child = {}
    for j in urdf.robot.joints:
        parent_map[j.child] = j.parent
        joint_by_child[j.child] = j
        
    if target_link not in parent_map and target_link != urdf.base_link:
        print(f"Error: Link '{target_link}' not found in the URDF tree.")
        return
        
    # Backtrack to find the full path from root
    curr = target_link
    chain_links = [curr]
    while curr in parent_map:
        curr = parent_map[curr]
        chain_links.append(curr)
        
    # Reverse to go from root -> target
    chain_links.reverse()
    
    print(f"Kinematic Chain from Root ({chain_links[0]}) to '{target_link}':\n")
    print("="*60)
    
    actual_chain = []
    
    # Print the identified sequence of links and joints
    for i, link_name in enumerate(chain_links):
        print(f"[Link] {link_name}")
        
        # If there is a next link, print the joint that connects to it
        if i < len(chain_links) - 1:
            child_link = chain_links[i+1]
            joint = joint_by_child[child_link]
            
            # Use Pinocchio to accurately determine degrees of freedom (nv)
            nv = 0
            if model.existJointName(joint.name):
                j_id = model.getJointId(joint.name)
                nv = model.joints[j_id].nv
                
            is_actuated = (nv > 0)
            actuated_text = "Yes" if is_actuated else "No"
            
            actual_chain.append({
                "link": link_name,
                "joint": joint.name,
                "type": joint.type,
                "actuated": is_actuated,
                "dofs": nv
            })
            
            # Print joint details
            print(f"   |")
            print(f"   +-- [Joint] {joint.name}")
            print(f"   |     Type: {joint.type}")
            print(f"   |     Actuated: {actuated_text}")
            if is_actuated:
                print(f"   |     DOFs: {nv}")
            print(f"   |")
        else:
            actual_chain.append({
                "link": link_name,
                "joint": None,
                "type": None,
                "actuated": None,
                "dofs": None
            })

    print("="*60)

    # Compare against expected tested chains and find the best match
    best_chain_name = None
    best_ratio = -1
    best_opcodes = None
    best_exp_chain = None

    act_links = [item["link"] for item in actual_chain]

    for chain_name, exp_chain in STANDARD_CHAINS.items():
        exp_links = [item["link"] for item in exp_chain]
        sm = difflib.SequenceMatcher(None, exp_links, act_links)
        ratio = sm.ratio()
        if ratio > best_ratio:
            best_ratio = ratio
            best_opcodes = sm.get_opcodes()
            best_exp_chain = exp_chain
            best_chain_name = chain_name

    differences = []
    
    for tag, i1, i2, j1, j2 in best_opcodes:
        if tag == 'equal':
            for idx in range(i2 - i1):
                exp = best_exp_chain[i1 + idx]
                act = actual_chain[j1 + idx]
                if act["joint"] != exp["joint"]:
                    differences.append(f"- In link '{act['link']}': Expected joint '{exp['joint']}', found '{act['joint']}'.")
                elif act["joint"] is not None:
                    if act["type"] != exp["type"]:
                        differences.append(f"- In joint '{act['joint']}': Expected type '{exp['type']}', found '{act['type']}'.")
                    if act["actuated"] != exp["actuated"]:
                        act_exp = 'Yes' if exp['actuated'] else 'No'
                        act_act = 'Yes' if act['actuated'] else 'No'
                        differences.append(f"- In joint '{act['joint']}': Expected actuation '{act_exp}', found '{act_act}'.")
                    if act["dofs"] != exp["dofs"]:
                        differences.append(f"- In joint '{act['joint']}': Expected DOFs {exp['dofs']}, found {act['dofs']}.")
        elif tag == 'insert':
            for j in range(j1, j2):
                j_name = actual_chain[j]['joint']
                differences.append(f"- Inserted link '{actual_chain[j]['link']}' (Joint: '{j_name if j_name else 'None'}').")
        elif tag == 'delete':
            for i in range(i1, i2):
                differences.append(f"- Missing expected link '{best_exp_chain[i]['link']}'.")
        elif tag == 'replace':
            # Identify close matches up to the length overlap, and treat the remainder as inserts/deletes
            max_len = max(i2 - i1, j2 - j1)
            for idx in range(max_len):
                if i1 + idx < i2 and j1 + idx < j2:
                    exp = best_exp_chain[i1 + idx]
                    act = actual_chain[j1 + idx]
                    differences.append(f"- Link '{act['link']}' is a close match to expected '{exp['link']}'. Differences:")
                    if act["joint"] != exp["joint"]:
                        differences.append(f"    > Expected joint '{exp['joint']}', found '{act['joint']}'.")
                    elif act["joint"] is not None:
                        if act["type"] != exp["type"]:
                            differences.append(f"    > Joint type: Expected '{exp['type']}', found '{act['type']}'.")
                        if act["actuated"] != exp["actuated"]:
                            act_exp = 'Yes' if exp['actuated'] else 'No'
                            act_act = 'Yes' if act['actuated'] else 'No'
                            differences.append(f"    > Joint actuation: Expected '{act_exp}', found '{act_act}'.")
                        if act["dofs"] != exp["dofs"]:
                            differences.append(f"    > Joint DOFs: Expected {exp['dofs']}, found {act['dofs']}.")
                elif i1 + idx < i2:
                    differences.append(f"- Missing expected link '{best_exp_chain[i1 + idx]['link']}'.")
                elif j1 + idx < j2:
                    j_name = actual_chain[j1 + idx]['joint']
                    differences.append(f"- Inserted link '{actual_chain[j1 + idx]['link']}' (Joint: '{j_name if j_name else 'None'}').")

    if differences:
        print(f"\nWARNING: The kinematic chain in {urdf_path} differs from the '{best_chain_name}' kinematic chain that has been tested in the following ways:")
        for diff in differences:
            print(f"  {diff}")
        print()
        return False
    else:
        print(f"\nSUCCESS: The kinematic chain in {urdf_path} matches the '{best_chain_name}' kinematic chain that has been tested.")
        return True

if __name__ == "__main__":

    main()
