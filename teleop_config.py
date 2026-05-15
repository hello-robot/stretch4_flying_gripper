import argparse
import sys

import stretch4_body.robot.robot_client as rc
import stretch4_body.robot.robot as rb
from stretch4_body.core.robot_params import RobotParams
from kinematic_controller import KinematicController

def get_base_parser(description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--speed", choices=['low', 'medium', 'high', 'max'], default='medium', help="Speeds of the joints")
    parser.add_argument("--strength", choices=['low', 'medium', 'high'], default='medium', help="Strengths of the joints")
    parser.add_argument("-d", "--direct", action="store_true", help="Use direct API (no server)")
    import os
    default_urdf = os.path.join(os.path.dirname(os.path.abspath(__file__)), "urdfs", "calder_ik.urdf")
    parser.add_argument("--urdf", type=str, default=default_urdf, help="Path to URDF file")
    parser.add_argument("--disable_extended_yaw", action="store_true", help="Disable rotating mobile base to extend wrist yaw range")
    parser.add_argument("--disable_flipped_wrist", action="store_true", help="Disable the flipped mode. The flipped mode is required for use with Calder and Dali when using the default specialized IK URDF derived from a Calder URDF (./urdfs/calder_ik.urdf).")
    parser.add_argument("--arm_blend_margin_extension", type=float, default=0.20, help="Margin range (meters) to seamlessly blend Cartesian movements from telescoping arm to mobile base as it approaches full extension.")
    parser.add_argument("--arm_blend_power_extension", type=float, default=2.0, help="Non-linear exponent to curve the blending penalty during arm extension (e.g. 1.0 is linear, 2.0 is quadratic onset).")
    parser.add_argument("--arm_blend_margin_retraction", type=float, default=0.05, help="Margin range (meters) to seamlessly blend Cartesian movements from telescoping arm to mobile base as it approaches full retraction.")
    parser.add_argument("--yaw_blend_margin", type=float, default=0.8, help="Margin range (radians) to seamlessly blend Cartesian rot-tracking from wrist yaw to the mobile base rotation as it approaches orientation limits.")
    parser.add_argument("--disable_retract_arm_at_extension", action="store_true", help="Disable retracting the arm slowly back to a target extension when it reaches full extension.")
    parser.add_argument("--retraction_ratio", type=float, default=0.25, help="Ratio of retraction speed relative to forward base speed for retract-at-extension mode (e.g. 0.25).")
    parser.add_argument("--retract_target_extension", type=float, default=0.20, help="Target extension length (meters) to retract to during the retract-at-extension mode (e.g. 0.20).")
    parser.add_argument("--mode4_max_arm_extension", type=float, default=0.48, help="Maximum allowed extension (meters) for the telescoping arm when using Control Mode 4. Limits the reach to prevent over-extension or to maintain stability.")
    return parser

def initialize_teleop_hardware(args):
    speed_mapping = {
        'low': 'slow',
        'medium': 'default',
        'high': 'fast', 
        'max': 'max'
    }
    
    strength_mapping = {
        'low': 'high_sensitivity_manipulation',
        'medium': 'default',
        'high': 'strong_manipulation'
    }
    
    if args.direct:
        robot = rb.Robot()
    else:
        robot = rc.RobotClient()
        
    if not robot.startup():
        print("Failed to start robot connection")
        sys.exit(1)
        
    print(f"Setting Contact Sensitivity to: {strength_mapping[args.strength]}")
    robot.set_guarded_contact_sensitivity(strength_mapping[args.strength])
    
    params = RobotParams().get_params()[1]
    motion_prof = speed_mapping[args.speed]

    accel_vel_dict = {
        'accel_base_xy': params['omnibase']['motion'][motion_prof]['accel_xy_m'],
        'accel_base_w': params['omnibase']['motion'][motion_prof]['accel_w_r'],
        'accel_lift': params['lift']['motion']['max']['accel_m'],
        'accel_arm': params['arm']['motion']['max']['accel_m'],
        'accel_yaw': params['wrist_yaw']['motion'][motion_prof]['accel'],
        'accel_pitch': params['wrist_pitch']['motion'][motion_prof]['accel'],
        'accel_roll': params['wrist_roll']['motion'][motion_prof]['accel'],
        'vel_yaw': params['wrist_yaw']['motion'][motion_prof]['vel'],
        'vel_pitch': params['wrist_pitch']['motion'][motion_prof]['vel'],
        'vel_roll': params['wrist_roll']['motion'][motion_prof]['vel'],
        'vel_grip': params['stretch_gripper']['motion'][motion_prof]['vel'],
        'acc_grip': params['stretch_gripper']['motion'][motion_prof]['accel']
    }
    
    if args.speed == 'low':
        accel_vel_dict['gamepad_speed_trans'] = 0.05
        accel_vel_dict['gamepad_speed_rot'] = 0.4
    elif args.speed == 'medium':
        accel_vel_dict['gamepad_speed_trans'] = 0.15
        accel_vel_dict['gamepad_speed_rot'] = 0.5
    else:
        accel_vel_dict['gamepad_speed_trans'] = 0.25
        accel_vel_dict['gamepad_speed_rot'] = 1.0

    print("Initializing IK Controller...")
    try:
        ikin = KinematicController(
            urdf_path=args.urdf, 
            extended_yaw=not args.disable_extended_yaw, 
            arm_blend_margin_extension=args.arm_blend_margin_extension, 
            arm_blend_power_extension=args.arm_blend_power_extension,
            arm_blend_margin_retraction=args.arm_blend_margin_retraction,
            yaw_blend_margin=args.yaw_blend_margin, 
            retract_arm_at_extension=not args.disable_retract_arm_at_extension,
            retraction_ratio=args.retraction_ratio,
            retract_target_extension=args.retract_target_extension,
            mode4_max_arm_extension=args.mode4_max_arm_extension
        )
    except Exception as e:
        print(f"Failed to initialize IK controller: {e}")
        robot.stop()
        sys.exit(1)
        
    return robot, ikin, accel_vel_dict
