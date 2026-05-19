import numpy as np
import coal # Do not remove this import, it helps pin import correctly on some systems
import pinocchio as pin

def get_mode1_jacobian(model, data, q, gripper_frame_id, translation_joint_ids):
    """
    Computes a 3x5 Jacobian mapping velocities of the 5 joints to the linear
    velocity of the gripper expressed in its own frame (Mode 1).
    
    Mode 1 Control:
    - forward => gripper -Y-axis
    - left => gripper X-axis
    - up => gripper Z-axis
    
    Returns:
    J_mode1: 3x5 layout [v_forward, v_left, v_up]
    """
    # Jacobian in the gripper's LOCAL frame
    J_full = pin.computeFrameJacobian(model, data, q, gripper_frame_id, pin.ReferenceFrame.LOCAL)
    
    # Extract linear velocity components
    # Local axes (new URDF quick_connect_interface_link): X (left), Y (backward), Z (up)
    v_left_row = J_full[0, :]
    v_fwd_row = -J_full[1, :]
    v_up_row = J_full[2, :]
    
    # We want the output vector to align with: [v_forward, v_left, v_up]
    J_mode1_full = np.vstack([v_fwd_row, v_left_row, v_up_row])
    
    # Extract columns corresponding to the 5 translational DOFs
    cols = []
    for j_id in translation_joint_ids:
        idx_v = model.joints[j_id].idx_v
        nv = model.joints[j_id].nv
        cols.extend(range(idx_v, idx_v + nv))
        
    return J_mode1_full[:, cols]


def get_mode2_jacobian(model, data, q, gripper_frame_id, base_frame_id, translation_joint_ids):
    """
    Computes a 3x5 Jacobian mapping velocities to the linear velocity expressed
    in the projected frame (Mode 2).
    
    Mode 2 Control:
    - forward => projection of gripper's -Y-axis on horizontal plane
    - left => projection of gripper's X-axis on horizontal plane
    - up => base_link's Z-axis
    """
    # Velocity of point at the origin of gripper frame, resolved in WORLD axes
    J_world = pin.computeFrameJacobian(model, data, q, gripper_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    J_lin_world = J_world[:3, :]
    
    # Gripper's rotation matrix in the world
    T_gripper = data.oMf[gripper_frame_id]
    R_gripper = T_gripper.rotation
    
    gripper_left = R_gripper[:, 0] # X axis is left
    gripper_fwd = -R_gripper[:, 1] # -Y axis is forward
    
    # Project -Y-axis (forward) onto the horizontal plane (Z=0)
    fwd_dir = np.copy(gripper_fwd)
    fwd_dir[2] = 0.0
    norm_fwd = np.linalg.norm(fwd_dir)
    if norm_fwd > 1e-6:
        fwd_dir = fwd_dir / norm_fwd
    else:
        fwd_dir = np.array([1.0, 0.0, 0.0])
        
    # Project X-axis (left) onto the horizontal plane
    left_dir = np.copy(gripper_left)
    left_dir[2] = 0.0
    norm_left = np.linalg.norm(left_dir)
    if norm_left > 1e-6:
        left_dir = left_dir / norm_left
    else:
        left_dir = np.array([0.0, 1.0, 0.0])
        
    # up_dir is base_link's Z axis
    T_base = data.oMf[base_frame_id]
    up_dir = T_base.rotation[:, 2]
    
    # R_proj maps from WORLD to [Forward, Left, Up] components
    R_proj = np.vstack([fwd_dir, left_dir, up_dir])
    
    J_mode2_full = R_proj @ J_lin_world
    
    cols = []
    for j_id in translation_joint_ids:
        idx_v = model.joints[j_id].idx_v
        nv = model.joints[j_id].nv
        cols.extend(range(idx_v, idx_v + nv))
        
    return J_mode2_full[:, cols]


def solve_translational_ik(model, q, v_desired, J, translation_joint_ids, arm_blend_margin_extension=0.20, arm_blend_power_extension=2.0, arm_blend_margin_retraction=0.05, retract_state=None):
    """
    Solves for the 5 joint velocities using a weighted damped pseudo-inverse.
    Biases motion to use base rotation, lift, and arm extension.
    Uses an active-set directional penalty: if lift or arm approach their geometric limits,
    AND the solver commands them to move into the limit, they are penalized progressively,
    forcing base translation (X,Y) to compensate smoothly before hitting the wall.
    
    v_desired: [v_forward, v_left, v_up]
    J: 3x5 Jacobian mapping 5 DOFs to the above velocities.
    """
    # The linear distance from structural limits where blending between 
    # dependent joints (like arm/base) seamlessly ramps up
    lift_blend_margin_meters = arm_blend_margin_extension # 10 cm start blending by default
    
    W = np.array([50.0,  # Base X (discouraged)
                  50.0,  # Base Y (discouraged)
                  1.0,   # Base Theta (encouraged)
                  1.0,   # Lift (encouraged)
                  1.0])  # Arm (encouraged)
    
    # Internal method applying dynamic weightings
    def compute_v(weights, v_tgt=v_desired):
        W_inv = np.diag(1.0 / weights)
        lambda_damp = 1e-4
        J_W_JT = J @ W_inv @ J.T + lambda_damp * np.eye(3)
        J_pinv = W_inv @ J.T @ np.linalg.inv(J_W_JT)
        return J_pinv @ v_tgt

    # 1st Pass: Unconstrained preferred solver
    v_5dof = compute_v(W)
    
    # Active Set Enforcement (Continuous Redundancy Extractor)
    changed = False
    
    arm_col = None
    arm_val = 0.0
    arm_upper = 0.0
    
    for i, j_id in enumerate(translation_joint_ids):
        # We assume order is Base(nv=3), then Lift, then Arm. That maps to Base=0,1,2; Lift=3; Arm=4
        name = model.names[j_id]
        
        if name in ["lift_joint", "arm_l4_joint"]:
            col = 3 if name == "lift_joint" else 4
            if name == "arm_l4_joint":
                margin_lower = arm_blend_margin_retraction
                margin_upper = arm_blend_margin_extension
            else:
                margin_lower = lift_blend_margin_meters
                margin_upper = lift_blend_margin_meters
            
            idx_q = model.joints[j_id].idx_q
            val = q[idx_q]
            lower = model.lowerPositionLimit[idx_q]
            upper = model.upperPositionLimit[idx_q]
            
            if name == "arm_l4_joint":
                arm_col = col
                arm_val = val
                arm_upper = upper
            
            # Smoothly transition weight from nominal (1.0) to heavily penalized (1000.0) across the margin window
            interpolation_ratio = 0.0
            
            if val <= lower + margin_lower and v_5dof[col] < 0:
                interpolation_ratio = np.clip(((lower + margin_lower) - val) / margin_lower, 0.0, 1.0)
            elif val >= upper - margin_upper and v_5dof[col] > 0:
                interpolation_ratio = np.clip((val - (upper - margin_upper)) / margin_upper, 0.0, 1.0)
                if name == "arm_l4_joint":
                    interpolation_ratio = interpolation_ratio ** arm_blend_power_extension
                
            if interpolation_ratio > 0.0:
                # Linearly scale penalty 
                W[col] = 1.0 + (1000.0 - 1.0) * interpolation_ratio
                changed = True
                
    if retract_state is not None and retract_state.get('enabled', False) and arm_col is not None:
        v_arm_unconstrained = v_5dof[arm_col]
        
        # State machine updates
        if retract_state['is_retracting']:
            if np.linalg.norm(v_desired) < 1e-6 or v_arm_unconstrained < -1e-5:
                retract_state['is_retracting'] = False
        else:
            if v_arm_unconstrained > 1e-5 and arm_val >= arm_upper - 1e-4:
                retract_state['is_retracting'] = True

        if retract_state['is_retracting']:
            if arm_val > retract_state.get('target_extension', 0.20):
                forced_v_arm = -retract_state.get('retraction_ratio', 0.25) * v_arm_unconstrained
            else:
                forced_v_arm = 0.0
                
            v_desired_rem = v_desired - J[:, arm_col] * forced_v_arm
            
            W_retract = W.copy()
            W_retract[arm_col] = 1e6
            
            v_5dof = compute_v(W_retract, v_tgt=v_desired_rem)
            v_5dof[arm_col] = forced_v_arm
            changed = False

    # 2nd Pass: Constrained solve pushing velocities through re-weighted kinematic limits
    if changed:
        v_5dof = compute_v(W)
        
    return v_5dof

def get_mode4_jacobian(model, data, q, grasp_center_frame_id, base_frame_id, mode4_joint_ids):
    """
    Computes a 3xN Jacobian mapping velocities to the linear velocity expressed
    in the projected gripper frame (Mode 4).
    
    Mode 4 Control:
    - x => projected gripper's forward axis
    - y => projected gripper's left axis
    - z => gravity aligned up axis
    """
    J_world = pin.computeFrameJacobian(model, data, q, grasp_center_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    J_lin_world = J_world[:3, :]
    
    T_gripper = data.oMf[grasp_center_frame_id]
    R_gripper = T_gripper.rotation
    
    yaw = np.arctan2(R_gripper[1, 0], R_gripper[0, 0])
    R_proj = np.array([
        [np.cos(yaw), np.sin(yaw), 0],
        [-np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    J_mode4_full = R_proj @ J_lin_world
    
    # Zero out irrelevant columns for base translation and wrist joints
    for j_id in mode4_joint_ids:
        name = model.names[j_id]
        idx_v = model.joints[j_id].idx_v
        nv = model.joints[j_id].nv
        
        if name == "mobile_base_planar_joint":
            # Zero out Base X and Base Y
            J_mode4_full[:, idx_v] = 0.0
            J_mode4_full[:, idx_v+1] = 0.0
        elif name in ["wrist_yaw_joint", "wrist_pitch_joint", "wrist_roll_joint"]:
            for col in range(idx_v, idx_v + nv):
                J_mode4_full[:, col] = 0.0
    
    cols = []
    for j_id in mode4_joint_ids:
        idx_v = model.joints[j_id].idx_v
        nv = model.joints[j_id].nv
        cols.extend(range(idx_v, idx_v + nv))
        
    return J_mode4_full[:, cols]

def solve_mode4_ik(model, q, v_desired, J, mode4_joint_ids):
    """
    Solves for the joint velocities using a damped pseudo-inverse.
    Irrelevant DOFs have already been zeroed out in the Jacobian.
    """
    lambda_damp = 1e-4
    J_JT = J @ J.T + lambda_damp * np.eye(3)
    J_pinv = J.T @ np.linalg.inv(J_JT)
    
    v_opt = J_pinv @ v_desired
    
    return v_opt
