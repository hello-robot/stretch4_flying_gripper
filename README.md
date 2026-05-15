# Stretch 4 Flying Gripper Control

## Introduction

This repository provides a minimal example that enables gamepad control of the Stretch 4 mobile manipulator's gripper using the gripper's coordinate system. For the main gripper-centric control mode, the user can think of themselves as piloting the gripper to fly through the world. The controller enables them to do so without attempting to contol the robot's individual joints in a coordinated way.

### Motivation

The original motivation for this style of control was to support autonomous and teleoperative control via imagery provided by Stretch 4's gripper camera. Control with respect to the gripper's coordinate system results in consistent, interpretable changes to images from the gripper camera. In contrast, joint-space control results in dramatically different changes depending on the robot's current configuration. For example, if the gripper is rotated 90 degrees in yaw such that its direction is orthogonal to the extension direction of the telescoping arm, extending the telescoping arm results in sideways motion in the images instead of forward motion. 


### Control Method

The two available gripper-centric control modes primarily change the gripper's orientation by directly controlling the wrist's yaw, pitch and roll joints. Simultaneously, the position of the gripper is changed by controlling the position of the end of the telescoping arm with respect to the world. 

To control the end-of-arm position, the controller uses a Jacobian found via a specialized URDF to set the robot's joint velocities using weighted damped pseudo-inverse control. This Jacobian relates changes in the omnidirectional base, lift and telescoping arm (5 degrees of freedom) to the end-of-arm position (3 degrees of freedom). Importantly, it biases solutions to use rotation of the omnidirectional mobile base instead of translation, since rotation provides higher quality motion. 

### Joint Limits

Another notable aspect of the controller is how it uses redundancy and whole body motion to handle joint limits. If the telescoping arm extends near its maximum reach, the mobile base begins to help translate the gripper. Once the telescoping arm extends to its joint limit, the mobile base is fully responsible for translating the gripper. At this point, the arm also begins to slowly retract. Doing so gradually improves the quality of motion and increases the ability of the arm to perform high-quality motions without hitting its joint limit. Also, the robot will begin moving backward if the telescoping arm is fully retracted and the gripper is commanded to go backward. 

A similar approach is used when controlling the gripper's yaw angle. As the wrist's yaw angle approaches a joint limit the mobile base begins to move so as to continue the gripper's rotation around the wrist yaw's axis of rotation. 

## Installation

**Note: Installation of this repository as a package is NOT necessary to use it.** Simply cloning the repository and installing its dependencies allows you to run `stretch_gamepad_teleop_gripper.py` directly from the command line while in the repository's root directory without any further installation steps.

1. Copy or clone this repository to your Stretch 4 robot.
2. The core requirements for connecting to the robot (`stretch4_body`, `stretch4_urdf`) are typically already installed on Stretch 4 systems.
3. Install the unmet third-party dependencies (`numpy`, `pinocchio`, `yourdfpy`) using the included installation script:
```bash
./install_dependencies.sh
```
*(This script securely handles PEP 668 externally-managed environments natively using `--break-system-packages` if required).*

### Advanced: Using as a Python Package
In addition to running as standalone scripts, the repository is formatted using modern Python packaging standards (`pyproject.toml`). This allows it to be installed into other virtual environments or at the system level so that its internals (`teleop_config`, `kinematic_controller`, etc.) can be imported from code in other repositories without having to manage directories or subdirectories.

You can install it locally as an editable package:
```bash
pip install -e .
```
Or directly install it using:
```bash
pip install .
```

## Usage

You can begin teleoperating the robot by executing the main script:

```bash
python3 stretch_gamepad_teleop_gripper.py
```

The controller relies on a standard gamepad. Press the **Top Button (Y)** to toggle between the three available control modes dynamically while operating the robot.

### Quick Start: Flying the Gripper

When you first launch the script, the system defaults to **Mode #1**. The absolute easiest way to get started with this control scheme is:
1. **Aim:** Use the **Right Stick** (yaw and pitch) to physically point the gripper at a target in the world. 
2. **Fly:** Move the gripper forward to the target by pushing the **Left Stick** up.
3. **Correct:** While the gripper is flying toward the target, use the **Right analog stick** to continuously correct its direction on the fly.

### Control Modes & Gamepad Mapping

The controller relies on a standard gamepad (like an Xbox controller). Press the **Top Button (Y)** to toggle between the three available control modes dynamically.

#### Universal Actions (All Modes)

```text
       [Left Trigger]                 [Right Trigger]
     Dampen / Slow down           Modifier (Mode 3 only)
             |                              |
         ____|______________________________|____
        /                                        \
       /    _                        (Y) Toggle   \
      |   /   \                          (Y)       |
      |  | LS  |                     (X)     (B) -----> Open Gripper
      |   \ _ /                          (A)       |
      |                                     |      |
      |           _                   _     |      |
      |         _| |_               /   \   |      |
      |        |_   _|             | RS  |  v      |
       \         |_|                \ _ / Close Gripper/
        \      (D-Pad)                            /
         \_______________________________________/
          (LS = Left Stick, RS = Right Stick)
```

*   **Top Button (Y):** Cycle through control modes (1 -> 2 -> 3 -> 1).
*   **Bottom Button (A):** Close Gripper.
*   **Right Button (B):** Open Gripper.
*   **Left Trigger:** Proportional Speed Dampener. Squeezing this trigger progressively slows down all movements for fine-tuned precision.

---

#### Modes 1 & 2: Cartesian IK Controllers

These modes use the **Pinocchio inverse-kinematics solver** to automatically calculate the combinations of base, arm, and lift movements required to move the gripper through Cartesian space.

*   **Mode 1: Gripper Frame Relative ("Flying Gripper Control")**
    Control is entirely with respect to the gripper's *own* 3D coordinate system. Translating "forward" moves the gripper exactly where it is pointing. Look through the gripper camera to "pilot" it freely.
*   **Mode 2: Projected Base Frame Relative ("Camera Intuitive Control")**
    Locks translation to the horizontal floor plane. "Forward" moves the gripper in its forward direction projected onto the ground, preventing the robot from unintentionally digging the gripper into the floor or lifting up when pointing down.

**Gamepad Mappings (Modes 1 & 2):**
```text
[ LS (Left Stick) ]
  Up / Down: Translate Forward / Backward
  Left / Right: Translate Left / Right

[ D-Pad ]                                   [ RS (Right Stick) ]
  Up / Down: Translate Up / Down              Up / Down: Wrist Pitch
  Left / Right: Wrist Roll                    Left / Right: Wrist Yaw
```

---

#### Mode 3: Joint-Space Direct Control

A direct hardware mapping where the gamepad inputs instruct individual physical joints directly. This bypasses the Cartesian inverse kinematics solver entirely.

**Gamepad Mappings (Mode 3 - Standard):**
```text
[ LS (Left Stick) ]
  Up / Down: Base Forward / Backward
  Left / Right: Base Translate Left / Right

[ D-Pad ]                                   [ RS (Right Stick) ]
  Up / Down: Lift Up / Down                   Up / Down: Arm Extend / Retract
                                              Left / Right: Base Turn (Rotate)
```

**Gamepad Mappings (Mode 3 - While holding RIGHT TRIGGER):**
Holding the `Right Trigger` replaces several chassis controls with wrist controls.
```text
[ LS (Left Stick) ]
  Up / Down: Arm Extend / Retract
  Left / Right: Wrist Roll

[ D-Pad ]                                   [ RS (Right Stick) ]
  Up / Down: Lift Up / Down                   Up / Down: Wrist Pitch
                                              Left / Right: Wrist Yaw
```

## How It Works

A key aspect of developing these controllers was making the most of the two redundant degrees of freedom (DOF). The Stretch 4 has 8 controllable DOFs, while the gripper's pose only requires 6 DOFs. 

The two new gripper-centric controllers (Modes 1 and 2) use a specialized URDF. This URDF includes a virtual joint that mathematically represents the omnidirectional mobile base. 

To map Cartesian intent to joint velocities, the script utilizes the **Pinocchio** dynamics library. Pinocchio calculates the specialized Jacobian for the active controller, identifying how small changes to the 5 translational joints (3-DOF omnidirectional base, 1-DOF lift, 1-DOF arm) relate to the forward/backward, left/right, and up/down Cartesian changes.

To resolve the 2 extra degrees of redundancy, the Jacobian is passed through a **weighted damped pseudo-inverse control** calculation. The logic biases the use of mobile base rotation over mobile base translation, since base rotation results in higher quality motion. Specifically, the solver generally prohibits X/Y base translation. However, if you get close to the physical joint limits of the telescoping arm or the wrist yaw, the algorithm dynamically scales the penalty weights, and the omnidirectional mobile base begins translating to keep the gripper moving along your commanded vector.

### Specialized URDF Configuration

The accuracy of this controller is fundamentally tied to the specialized URDF. Without it, the Jacobian matrix would not account for the omnidirectional drive properly. The specialized URDF specifically introduces a **virtual planar joint** to mathematically model the mobile base's degrees of freedom.

If you would like to use a custom URDF for your specific robot model, you must generate an IK-compatible URDF using the Stretch 4 Urdf package.

1. Navigate to the Hello Robot official `stretch4_urdf` package.
2. Execute the generator script: [urdf_utils_generate_ik_urdfs.py](https://github.com/hello-robot/stretch4_urdf/blob/main/stretch4_urdf/urdf_utils_generate_ik_urdfs.py)
3. This script will output several files. Identify the specialized URDF containing the text `base_planar_ik` in its filename.
4. **Verify the URDF** by running the internal `check_kinematic_chain.py` tool. You can pass your URDF path to see if your chain differs physically or structurally from other officially tested kinematic chains:
   ```bash
   python3 check_kinematic_chain.py <your_generated_urdf>.urdf
   ```
5. Launch your teleop session by explicitly overriding the default URDF path in the arguments:
   ```bash
   python3 stretch_gamepad_teleop_gripper.py --urdf <your_generated_urdf>.urdf
   ```

**Important Notes for Calder and Dali Robots:**
*   By default, `stretch_gamepad_teleop_gripper.py` points to `/tmp/stretch_gamepad_teleop/gamepad_teleop_base_planar_ik.urdf` generated by `stretch4_urdf.generate_ik_urdfs()`. This repository has been successfully tested and works well with the Francis model of Stretch 4.
*   The default configuration automatically handles a known sign mismatch between the real-world pitch/roll wrist joints on the Calder and Dali hardware and the URDF mathematical model. If you use a different robot configuration without this mismatch, you may need to append the `--disable_flipped_wrist` argument to stop this software inversion. Also, always target the URDF matched to your specific hardware.
