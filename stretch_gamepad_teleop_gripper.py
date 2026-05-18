#!/usr/bin/env python3
import time
import sys
import numpy as np

import coal # Do not remove this import, it helps pin import correctly on some systems
import pinocchio as pin

from gamepad_mapper import GamepadMapper
# Kinematic resolution handled dynamically in teleop_config.py
import teleop_config

def main():
    parser = teleop_config.get_base_parser('Gripper-centric Teleop for Stretch')
    args = parser.parse_args()

    robot, ikin, accel_vel_dict = teleop_config.initialize_teleop_hardware(args)

    accel_base_xy = accel_vel_dict['accel_base_xy']
    accel_base_w = accel_vel_dict['accel_base_w']
    accel_lift = accel_vel_dict['accel_lift']
    accel_arm = accel_vel_dict['accel_arm']
    accel_yaw = accel_vel_dict['accel_yaw']
    accel_pitch = accel_vel_dict['accel_pitch']
    accel_roll = accel_vel_dict['accel_roll']
    
    vel_yaw = accel_vel_dict['vel_yaw']
    vel_pitch = accel_vel_dict['vel_pitch']
    vel_roll = accel_vel_dict['vel_roll']

    vel_grip = accel_vel_dict['vel_grip']
    acc_grip = accel_vel_dict['acc_grip']
    
    gamepad_speed_trans = accel_vel_dict['gamepad_speed_trans']
    gamepad_speed_rot = accel_vel_dict['gamepad_speed_rot']

    print("Initializing Gamepad Mapper...")
    try:
        gamepad = GamepadMapper()
    except Exception as e:
        print(f"Failed to connect to gamepad: {e}")
        robot.stop()
        sys.exit(1)
    
    control_mode = 1
    hz = 30.0
    dt = 1.0 / hz
    rate = time.time()
    
    gripper_close_pct = -60.0
    gripper_open_pct = 60.0
    
    print("====================================")
    print("Gripper-Centric Teleop Started")
    print("Press Top Button (Y) to Toggle Modes")
    print("Mode 1: Gripper Frame Relative")
    print("Mode 2: Projected Base Frame Relative")
    print("Ctrl+C to Quit")
    print("====================================")

    try:
        while True:
            cmd = gamepad.get_commands()
            
            if not args.direct:
                robot.pull_status()
                
            # Sync IK configuration with real robot
            pitch_sign_mult = 1.0 if args.disable_flipped_wrist else -1.0
            roll_sign_mult = -1.0 if args.disable_flipped_wrist else 1.0
            
            ikin.q[0] = robot.base.status['x']
            ikin.q[1] = robot.base.status['y']
            ikin.q[2] = np.cos(robot.base.status['theta'])
            ikin.q[3] = np.sin(robot.base.status['theta'])
            ikin.q[4] = robot.lift.status['pos']
            ikin.q[5] = robot.arm.status['pos']
            ikin.q[6] = robot.end_of_arm.status['wrist_yaw']['pos']
            ikin.q[7] = robot.end_of_arm.status['wrist_pitch']['pos'] * pitch_sign_mult
            ikin.q[8] = robot.end_of_arm.status['wrist_roll']['pos'] * roll_sign_mult
            
            pin.forwardKinematics(ikin.model, ikin.data, ikin.q)
            pin.updateFramePlacements(ikin.model, ikin.data)
            
            if not cmd:
                robot.base.set_velocity(0, 0, 0, accel_base_xy*2, accel_base_w*2)
                robot.lift.set_velocity(0, a_m=accel_lift)
                robot.arm.set_velocity(0, a_m=accel_arm)
                robot.end_of_arm.move_by('wrist_yaw', 0)
                robot.end_of_arm.move_by('wrist_pitch', 0)
                robot.end_of_arm.move_by('wrist_roll', 0)
                robot.push_command()
                time.sleep(dt)
                continue
            
            if cmd['toggle']:
                control_mode = (control_mode % 3) + 1
                mode_names = {
                    1: "Gripper Frame Relative (IK)",
                    2: "Projected Base Frame Relative (IK)",
                    3: "Joint-Space Direct Control"
                }
                print(f"--> Switched to Mode {control_mode}: {mode_names[control_mode]}")

            if cmd['grip'] == "OPEN":
                robot.end_of_arm.move_by('stretch_gripper', gripper_open_pct, vel_grip, acc_grip)
            elif cmd['grip'] == "CLOSE":
                robot.end_of_arm.move_by('stretch_gripper', gripper_close_pct, vel_grip, acc_grip)

            # Apply left-trigger proportional dampening: halving speed at full press
            left_trigger = cmd.get('left_trigger', 0.0)
            speed_multiplier = 1.0 - (0.5 * left_trigger)


            if control_mode == 3:
                # Direct joint space mapping ignoring kinematics solver
                v_vel = np.zeros(8)
                
                # Lift (D-Pad Up/Down)
                v_vel[3] = cmd['v_desired'][2] * gamepad_speed_trans * speed_multiplier
                
                # Right Trigger overrides standard chassis controls to Wrist controls
                if cmd.get('right_trigger', 0.0) > 0.1:
                    # Arm Extend/Retract (Left Stick Y) overrides Base Forward/Backward
                    v_vel[4] = cmd['v_desired'][0] * gamepad_speed_trans * speed_multiplier
                    
                    # Wrist Roll (Left Stick X) - Flipped to match intuitive rotation
                    v_vel[7] = -cmd['v_desired'][1] * gamepad_speed_rot * speed_multiplier
                    
                    # Wrist Pitch (Right Stick Y)
                    v_vel[6] = cmd['rot_change'][1] * gamepad_speed_rot * speed_multiplier
                    
                    # Wrist Yaw (Right Stick X)
                    v_vel[5] = cmd['rot_change'][0] * gamepad_speed_rot * speed_multiplier
                else:
                    # Base Translation Forward/Backward (Left Stick Y)
                    v_vel[0] = cmd['v_desired'][0] * gamepad_speed_trans * speed_multiplier
                    
                    # Base Translation Left/Right (Left Stick X)
                    v_vel[1] = cmd['v_desired'][1] * gamepad_speed_trans * speed_multiplier
                    
                    # Base Rotation (Right Stick X)
                    v_vel[2] = cmd['rot_change'][0] * gamepad_speed_rot * speed_multiplier
                    
                    # Arm (Right Stick Y)
                    v_vel[4] = cmd['rot_change'][1] * gamepad_speed_trans * speed_multiplier
                
                v = v_vel * dt

                
            else:
                v_desired = np.array(cmd['v_desired']) * gamepad_speed_trans * speed_multiplier * dt
                rot_change = np.array(cmd['rot_change']) * gamepad_speed_rot * speed_multiplier * dt
                
                # v maps to joint displacements [delta_q_base_x, delta_q_base_y, delta_q_base_theta, delta_q_lift, delta_q_arm, delta_q_yaw, delta_q_ pitch, delta_q_roll]
                v, _ = ikin.compute_ik_step(v_desired, rot_change, control_mode)
                
                # Scale from displacement `v` back to continuous velocity by dividing by `dt`
                v_vel = v / dt
                
            if np.any(v != 0):         
                # Command actual hardware
                robot.base.set_velocity(v_vel[0], v_vel[1], v_vel[2], a_m=accel_base_xy, a_r=accel_base_w)
                robot.lift.set_velocity(v_vel[3], a_m=accel_lift)
                robot.arm.set_velocity(v_vel[4], a_m=accel_arm)
                
                # Smoothing move_by control commands using a high lookahead targeting horizon
                # and reduced acceleration to seamlessly simulate velocity tracking without stopping abruptly
                lookahead = 10.0
                v_yaw_cmd = min(abs(v_vel[5]), vel_yaw)
                v_pitch_cmd = min(abs(v_vel[6]), vel_pitch)
                v_roll_cmd = min(abs(v_vel[7]), vel_roll)
                
                robot.end_of_arm.move_by('wrist_yaw', v[5] * lookahead, v_yaw_cmd, accel_yaw * 0.5)
                robot.end_of_arm.move_by('wrist_pitch', v[6] * pitch_sign_mult * lookahead, v_pitch_cmd, accel_pitch * 0.5)
                robot.end_of_arm.move_by('wrist_roll', v[7] * roll_sign_mult * lookahead, v_roll_cmd, accel_roll * 0.5)
            else:
                robot.base.set_velocity(0, 0, 0, accel_base_xy*2, accel_base_w*2)
                robot.lift.set_velocity(0, a_m=accel_lift)
                robot.arm.set_velocity(0, a_m=accel_arm)
                robot.end_of_arm.move_by('wrist_yaw', 0)
                robot.end_of_arm.move_by('wrist_pitch', 0)
                robot.end_of_arm.move_by('wrist_roll', 0)
                
            robot.push_command()
            
            # Use dynamic sleep to maintain loop frequency
            sleep_time = rate + dt - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            rate = time.time()
            
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        robot.base.set_velocity(0, 0, 0)
        robot.lift.set_velocity(0)
        robot.arm.set_velocity(0)
        robot.end_of_arm.move_by('wrist_yaw', 0)
        robot.end_of_arm.move_by('wrist_pitch', 0)
        robot.end_of_arm.move_by('wrist_roll', 0)
        robot.push_command()
        
        gamepad.stop()
        robot.stop()

if __name__ == '__main__':
    main()
