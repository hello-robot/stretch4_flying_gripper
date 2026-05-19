import numpy as np
import coal # Do not remove this import, it helps pin import correctly on some systems
import pinocchio as pin

from stretch4_flying_gripper.kinematic_tools import get_mode1_jacobian, get_mode2_jacobian, solve_translational_ik, get_mode4_jacobian, solve_mode4_ik

class KinematicController:
    """
    Hardware-agnostic inverse kinematics controller for the stretch platform.
    This strictly evaluates Pinocchio transformations to map Cartesian commands
    into joint space velocities (`v`) and integrates them into configuration positions (`q`).
    """
    def __init__(self, urdf_path, extended_yaw=False, arm_blend_margin_extension=0.20, arm_blend_power_extension=2.0, arm_blend_margin_retraction=0.05, yaw_blend_margin=0.8, retract_arm_at_extension=False, retraction_ratio=0.25, retract_target_extension=0.20, mode4_max_arm_extension=0.48):
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.model = pin.buildModelFromUrdf(urdf_path)
            
        self.extended_yaw = extended_yaw
        self.mode4_max_arm_extension = mode4_max_arm_extension
        self.arm_blend_margin_extension = arm_blend_margin_extension
        self.arm_blend_power_extension = arm_blend_power_extension
        self.arm_blend_margin_retraction = arm_blend_margin_retraction
        self.yaw_blend_margin = yaw_blend_margin
        self.retract_state = {
            'enabled': retract_arm_at_extension,
            'is_retracting': False,
            'retraction_ratio': retraction_ratio,
            'target_extension': retract_target_extension
        }
        self.data = self.model.createData()
        self.q = pin.neutral(self.model)
        
        self.translation_joints = ["mobile_base_planar_joint", "lift_joint", "arm_l4_joint"]
        self.trans_jids = [self.model.getJointId(n) for n in self.translation_joints if self.model.existJointName(n)]
        
        self.rotation_joints = ["wrist_yaw_joint", "wrist_pitch_joint", "wrist_roll_joint"]
        self.rot_jids = [self.model.getJointId(n) for n in self.rotation_joints if self.model.existJointName(n)]
        
        if len(self.rot_jids) > 0:
            w_id = self.rot_jids[0]
            idx_q = self.model.joints[w_id].idx_q
            limit = 1.91986 # 110 degrees
            upper = self.model.upperPositionLimit[idx_q]
            lower = self.model.lowerPositionLimit[idx_q]
            self.model.upperPositionLimit[idx_q] = min(upper, limit)
            self.model.lowerPositionLimit[idx_q] = max(lower, -limit)
        
        self.gripper_frame_id = self.model.getFrameId("tool_attachment_site_link")
        self.base_frame_id = self.model.getFrameId("base_link")
        
        if self.model.existFrame("grasp_center_link"):
            self.grasp_center_frame_id = self.model.getFrameId("grasp_center_link")
        else:
            self.grasp_center_frame_id = self.gripper_frame_id
            
        self.mode4_jids = self.trans_jids.copy()
        for rot_name in ["wrist_yaw_joint", "wrist_pitch_joint"]:
            if self.model.existJointName(rot_name):
                self.mode4_jids.append(self.model.getJointId(rot_name))
        
        if self.model.existFrame("wrist_link"):
            self.wrist_frame_id = self.model.getFrameId("wrist_link")
        else:
            self.wrist_frame_id = self.gripper_frame_id
        
        # Precompute initial kinematics
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)

    def compute_ik_step(self, v_desired, rot_change, control_mode=1):
        """
        Takes raw continuous velocities and applies them to the robot state.

        Args:
            v_desired (np.ndarray): Target Cartesian translation (e.g Forward, Left, Up) in robot frames.
            rot_change (np.ndarray): Target Rotational speeds (e.g Yaw, Pitch, Roll).
            control_mode (int): 1 = Gripper Frame relative, 2 = Projected Base Frame relative.
            
        Returns:
            np.ndarray: Evaluated joint velocities strictly applied to the current frame
            np.ndarray: The resulting new configuration vector `q`
        """
        v = np.zeros(self.model.nv)
        
        if control_mode == 4:
            if np.any(v_desired != 0):
                J = get_mode4_jacobian(self.model, self.data, self.q, self.grasp_center_frame_id, self.base_frame_id, self.mode4_jids)
                v_mode4 = solve_mode4_ik(self.model, self.q, v_desired, J, self.mode4_jids)
                
                c = 0
                for j_id in self.mode4_jids:
                    idx_v = self.model.joints[j_id].idx_v
                    nv = self.model.joints[j_id].nv
                    v[idx_v : idx_v+nv] = v_mode4[c : c+nv]
                    c += nv
            
        else:
            # 0. Extended Yaw Redundancy Extraction
            rc_yaw = rot_change[0]
            base_yaw_velocity = 0.0
            
            if self.extended_yaw and rc_yaw != 0.0 and len(self.rot_jids) > 0:
                w_id = self.rot_jids[0]
                idx_q = self.model.joints[w_id].idx_q
                w_q = self.q[idx_q]
                w_upper = self.model.upperPositionLimit[idx_q]
                w_lower = self.model.lowerPositionLimit[idx_q]
                
                margin = self.yaw_blend_margin # ~45 deg default
                if w_q >= w_upper - 1e-4 and rc_yaw > 0:
                    base_yaw_velocity = rc_yaw
                    rot_change[0] = 0.0
                elif w_q > w_upper - margin and rc_yaw > 0:
                    ratio = np.clip((w_q - (w_upper - margin)) / margin, 0.0, 1.0)
                    base_yaw_velocity = rc_yaw * ratio
                    rot_change[0] = rc_yaw * (1.0 - ratio)
                    
                elif w_q <= w_lower + 1e-4 and rc_yaw < 0:
                    base_yaw_velocity = rc_yaw
                    rot_change[0] = 0.0
                elif w_q < w_lower + margin and rc_yaw < 0:
                    ratio = np.clip(((w_lower + margin) - w_q) / margin, 0.0, 1.0)
                    base_yaw_velocity = rc_yaw * ratio
                    rot_change[0] = rc_yaw * (1.0 - ratio)
            
            # 1. Process Rotations Directly (Wrist Control)
            if np.any(rot_change != 0):
                for i, rc in enumerate(rot_change):
                    if rc != 0 and i < len(self.rot_jids):
                        j_id = self.rot_jids[i]
                        idx_v = self.model.joints[j_id].idx_v
                        v[idx_v] = rc
                        
            # 2. Process user translation via Pseudo-Inverse Jacobian
            v_5dof = np.zeros(5)
            if np.any(v_desired != 0):
                if control_mode == 1:
                    J = get_mode1_jacobian(self.model, self.data, self.q, self.gripper_frame_id, self.trans_jids)
                else:
                    J = get_mode2_jacobian(self.model, self.data, self.q, self.gripper_frame_id, self.base_frame_id, self.trans_jids)
                v_5dof = solve_translational_ik(self.model, self.q, v_desired, J, self.trans_jids, arm_blend_margin_extension=self.arm_blend_margin_extension, arm_blend_power_extension=self.arm_blend_power_extension, arm_blend_margin_retraction=self.arm_blend_margin_retraction, retract_state=self.retract_state)
            else:
                self.retract_state['is_retracting'] = False
                
            # 3. Extended Yaw Target Compensation (strictly at wrist_link origin)
            v_comp_5dof = np.zeros(5)
            if base_yaw_velocity != 0.0:
                J_wrist = pin.computeFrameJacobian(self.model, self.data, self.q, self.wrist_frame_id, pin.LOCAL_WORLD_ALIGNED)[:3, :]
                J_5dof = np.zeros((3, 5))
                c = 0
                for j_id in self.trans_jids:
                    idx_v = self.model.joints[j_id].idx_v
                    nv = self.model.joints[j_id].nv
                    J_5dof[:, c : c+nv] = J_wrist[:, idx_v : idx_v+nv]
                    c += nv
                    
                # Compute arc translation induced solely by the base turning at base_yaw_velocity
                v_induced = J_5dof[:, 2] * base_yaw_velocity
                
                # Counteract by forcing X/Y/Z/R nulling speeds
                idx_4dof = [0, 1, 3, 4]
                J_4dof = J_5dof[:, idx_4dof]
                v_comp_4dof = np.linalg.pinv(J_4dof) @ (-v_induced)
                
                v_comp_5dof[idx_4dof] = v_comp_4dof
                v_comp_5dof[2] = base_yaw_velocity
                
            # 4. Enforce Safety Velocity Bounds
            v_5dof_total = v_5dof + v_comp_5dof
            
            # Explicit truncation that guarantees base stabilization while preserving exact mathematical path integration curvature
            scale = 1.0
            base_lin = np.linalg.norm(v_5dof_total[0:2])
            if base_lin > 0.15:
                scale = min(scale, 0.15 / base_lin)
                
            if abs(v_5dof_total[2]) > 0.3:
                scale = min(scale, 0.3 / abs(v_5dof_total[2]))
                
            if scale < 1.0:
                v_5dof_total[0:2] *= scale
                v_5dof_total[2] *= scale
                
                if self.extended_yaw:
                    v[self.model.joints[self.rot_jids[0]].idx_v] *= scale
                
            # 5. Integrate Solved 5-DOF Base state into velocity vector
            c = 0
            for j_id in self.trans_jids:
                idx_v = self.model.joints[j_id].idx_v
                nv = self.model.joints[j_id].nv
                v[idx_v : idx_v+nv] = v_5dof_total[c : c+nv]
                c += nv
                
            # 6. Wrist Yaw Orientation Compensation
            v_theta = v_5dof[2] 
            if v_theta != 0.0 and self.model.existJointName("wrist_yaw_joint"):
                wrist_yaw_id = self.model.getJointId("wrist_yaw_joint")
                idx_v_yaw = self.model.joints[wrist_yaw_id].idx_v
                v[idx_v_yaw] -= v_theta
                
        # 4. Integrate Configuration Space Constraints
        if np.any(v != 0):
            self.q = pin.integrate(self.model, self.q, v)
        
            # Strict Positional Limitation Enforcement (Clamps only 1-DOF arrays)
            for j_id in range(1, len(self.model.joints)):
                joint = self.model.joints[j_id]
                if joint.nv == 1:
                    idx_q = joint.idx_q
                    
                    lower = self.model.lowerPositionLimit[idx_q]
                    upper = self.model.upperPositionLimit[idx_q]
                    
                    if control_mode == 4:
                        if self.model.names[j_id] == "arm_l4_joint":
                            upper = min(upper, self.mode4_max_arm_extension)

                    self.q[idx_q] = np.clip(self.q[idx_q], lower, upper)
        
            # Re-compile absolute kinematic graph structure
            pin.forwardKinematics(self.model, self.data, self.q)
            pin.updateFramePlacements(self.model, self.data)
            
        return v, self.q

    def get_configuration(self):
        """Returns the internal current `q` configuration array."""
        return self.q
