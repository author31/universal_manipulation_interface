import pybullet as p
import numpy as np
import os

from .. import config 

def inspect_robot(robot_id, client_id):
    """
    Prints detailed information for each joint, including its type.
    """
    # Create a mapping from PyBullet's integer joint types to human-readable strings
    joint_type_map = {
        p.JOINT_REVOLUTE: "JOINT_REVOLUTE",   # Rotates around an axis (like an elbow)
        p.JOINT_PRISMATIC: "JOINT_PRISMATIC", # Slides along an axis (like a piston)
        p.JOINT_SPHERICAL: "JOINT_SPHERICAL", # 3D ball-and-socket
        p.JOINT_PLANAR: "JOINT_PLANAR",     # Moves in a 2D plane
        p.JOINT_FIXED: "JOINT_FIXED"        # A rigid, unmoving connection
    }

    num_joints = p.getNumJoints(robot_id, physicsClientId=client_id)
    print(f"🤖 Robot '{robot_id}' Detailed Joint Info ({num_joints} total):")
    print("--------------------------------------------------")
    
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i, physicsClientId=client_id)
        idx = joint_info[0]
        name = joint_info[1].decode('utf-8')
        j_type_int = joint_info[2]
        j_type_str = joint_type_map.get(j_type_int, f"UNKNOWN_TYPE_{j_type_int}")
        
        print(f"Index: {idx:<2} | Name: {name:<20} | Type: {j_type_str}")
    
    print("--------------------------------------------------")


class Franka:
    """
    A wrapper for the Franka Emika Panda robot in a PyBullet simulation.
    This class handles loading the robot, finding joint/link indices,
    and provides an interface for control and state querying.
    """
    def __init__(self, client_id, base_pos=(0.5, 0.5, 0.625), base_orn=(0, 0, 0, 1)):
        self.client_id = client_id
        self.base_pos = np.array(base_pos)
        self.base_orn = np.array(base_orn)

        # Load the robot URDF
        asset_path = os.path.join(config.get_meshdir(), "franka_panda/robots/franka_panda_umi.urdf")
        self.robot_id = p.loadURDF(
            fileName=asset_path,
            basePosition=self.base_pos,
            baseOrientation=p.getQuaternionFromEuler([0, 0, 4.7]),
            useFixedBase=True,
        )

        # Build mappings from joint/link names to PyBullet indices
        self._build_mappings()
        
        # Define indices for controllable arm joints and fingers
        self.arm_joint_indices = []
        self.finger_joint_indices = []
        self.hand_link_index = self.link_map['panda_hand']

        for i in range(p.getNumJoints(self.robot_id, physicsClientId=self.client_id)):
            joint_info = p.getJointInfo(self.robot_id, i, physicsClientId=self.client_id)
            joint_name = joint_info[1].decode('utf-8')
            joint_type = joint_info[2]

            if joint_type != p.JOINT_FIXED:
                if 'panda_joint' in joint_name and 'finger' not in joint_name:
                    self.arm_joint_indices.append(i)
                elif 'finger' in joint_name and 'tip' not in joint_name:
                    self.finger_joint_indices.append(i)

        self.initial_joint_pos = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        self.grip_open_state = 0.05
        self.grip_close_state = 0.0

    def _build_mappings(self):
        """Create dictionaries mapping joint/link names to their PyBullet indices."""
        self.joint_map = {}
        self.link_map = {}
        num_joints = p.getNumJoints(self.robot_id, physicsClientId=self.client_id)

        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i, physicsClientId=self.client_id)
            joint_name = joint_info[1].decode('utf-8')
            link_name = joint_info[12].decode('utf-8')
            
            self.joint_map[joint_name] = i
            self.link_map[link_name] = i

    def get_state(self):
        """
        Gets the current state of the robot.
        Replaces IsaacGym's tensor-based state acquisition.
        """
        # Get end-effector (hand) state
        link_state = p.getLinkState(self.robot_id, self.hand_link_index, computeLinkVelocity=1, physicsClientId=self.client_id)
        hand_pos = link_state[4]
        hand_rot = link_state[5]
        hand_vel_p = link_state[6]
        hand_vel_r = link_state[7]

         # Get finger states
        finger1_state = p.getJointState(self.robot_id, self.finger_joint_indices[0], physicsClientId=self.client_id)
        finger2_state = p.getJointState(self.robot_id, self.finger_joint_indices[1], physicsClientId=self.client_id)

        # Get arm joint states
        arm_joint_states = p.getJointStates(self.robot_id, self.arm_joint_indices, physicsClientId=self.client_id)
        arm_joint_pos = [state[0] for state in arm_joint_states]

        state_dict = {
            'hand_pos': np.array(hand_pos),
            'hand_rot': np.array(hand_rot),
            'hand_vel_p': np.array(hand_vel_p),
            'hand_vel_r': np.array(hand_vel_r),
            'l_finger_pos': finger1_state[0],
            'r_finger_pos': finger2_state[0],
            'l_finger_vel': finger1_state[1],
            'r_finger_vel': finger2_state[1],
            'arm_joint_pos': np.array(arm_joint_pos)
        }
        return state_dict

    def get_camera_image(self, 
                         fov=60, 
                         near=0.01, 
                         far=2.0, 
                         img_width=640, 
                         img_height=480, 
                         link_name="gopro_link"):
        """
        Capture an RGB image from a camera attached to a given link.

        Args:
            fov (float): Field of view in degrees.
            near (float): Near clipping plane.
            far (float): Far clipping plane.
            img_width (int): Image width.
            img_height (int): Image height.
            link_name (str): Link to attach camera to.
            offset (tuple): Offset from link origin (camera position).
            target_offset (tuple): Offset relative to camera for look-at target.
        """
        link_index = self.link_map[link_name]
        link_state = p.getLinkState(self.robot_id, link_index, computeForwardKinematics=True, physicsClientId=self.client_id)
        pos, orn = link_state[0], link_state[1]
        
        # this is fixed, it's really bad
        pos_offset = [0.05, 0, 0.1]
        rpy_offset = [0.785, -1.57, 0]

        orn_offset = p.getQuaternionFromEuler(rpy_offset)
        pos_cam, orn_cam = p.multiplyTransforms(pos, orn, pos_offset, orn_offset)
        rot_matrix_cam = p.getMatrixFromQuaternion(orn_cam)
        rot_matrix_cam = np.array(rot_matrix_cam).reshape(3, 3)
        forward_vector = rot_matrix_cam[:, 0]
        up_vector = rot_matrix_cam[:, 2]
        target_pos = np.add(pos_cam, forward_vector)

        view_matrix = p.computeViewMatrix(
            cameraEyePosition=pos_cam,
            cameraTargetPosition=target_pos,
            cameraUpVector=up_vector
        )

        proj_matrix = p.computeProjectionMatrixFOV(fov=155, aspect=1.0, nearVal=0.01, farVal=2.0)
        img = p.getCameraImage(width=img_width, height=img_height, viewMatrix=view_matrix, projectionMatrix=proj_matrix)

        return (
            np.reshape(img[2], (img_height, img_width, 4)).astype(np.uint8)
            .astype(np.uint8)
        )[:, :, :3]

    def reset(self):
        """Resets the robot's joints to their initial positions."""
        # Reset arm joints
        num_joints = p.getNumJoints(self.robot_id, physicsClientId=self.client_id)
        print(f"{num_joints=}")
        for i, joint_index in enumerate(self.arm_joint_indices):
            p.resetJointState(self.robot_id, joint_index, self.initial_joint_pos[i], physicsClientId=self.client_id)
        
        # Reset fingers to open state
        for joint_index in self.finger_joint_indices:
            p.resetJointState(self.robot_id, joint_index, self.grip_open_state, physicsClientId=self.client_id)

    def set_action(self, arm_action, gripper_action):
        """
        Applies actions to the robot's joints.
        
        Args:
            arm_action (np.ndarray): Target joint positions for the 7 arm DoFs.
            gripper_action (float): Target position for the gripper fingers (0 for closed, 0.05 for open).
        """
        # Control arm joints
        p.setJointMotorControlArray(
            bodyUniqueId=self.robot_id,
            jointIndices=self.arm_joint_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=arm_action,
            # PD gains can be tuned for better performance
            forces=[500] * len(self.arm_joint_indices)
        )

        # Control gripper fingers
        p.setJointMotorControlArray(
            bodyUniqueId=self.robot_id,
            jointIndices=self.finger_joint_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=[gripper_action] * 2,
            forces=[100] * 2
        )