import gym
from gym import spaces
import pybullet as p
import pybullet_data
import numpy as np
import os

from .robots.franka import Franka
from . import config

class FrankaAssemblyEnv(gym.Env):
    """
    Gym environment for a robot assembly task.
    This class follows the standard gym.Env interface.
    """
    metadata = {'render_modes': ['human', 'rgb_array'], 'render_fps': 240}

    def __init__(self, render_mode='human', render_size=224):
        super().__init__()
        self.render_mode = render_mode
        self.render_size = render_size
        
        # Initialize PyBullet physics client
        if self.render_mode == 'human':
            self.client_id = p.connect(p.GUI, options="--renderscale=2.0")
        else:
            self.client_id = p.connect(p.DIRECT)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0) # We will step manually
        p.resetDebugVisualizerCamera(
            cameraDistance=2, cameraYaw=0, cameraPitch=-28, cameraTargetPosition=[0, 0, 0]
        )

        # Define action and observation spaces
        # Action: 6 for arm joint positions, 1 for gripper
        self.action_space = spaces.Box(low=-1, high=1, shape=(7,), dtype=np.float32)

        # Observation: Dictionary with robot and object states
        self.observation_space = spaces.Dict({
            "image": spaces.Box(low=0, high=1, shape=(3,self.render_size, self.render_size), dtype=np.float32),
            "hand_pos": spaces.Box(low=-2, high=2, shape=(3,), dtype=np.float32),
        })
        self._load_scene()

    def _load_scene(self):
        """Loads the robot and objects into the simulation."""
        # Load table and ground plane
        plane_path = os.path.join(config.get_meshdir(), "plane/plane.urdf")
        p.loadURDF(plane_path, [0, 0, 0], physicsClientId=self.client_id)

        # For custom assets, provide full path
        table_path = os.path.join(config.get_meshdir(), "table/table.urdf")
        p.loadURDF(table_path, [0.5, 0, 0], useFixedBase=True, physicsClientId=self.client_id)
        
        # Load the robot
        self.robot = Franka(client_id=self.client_id)

    def reset(self, seed=None, options=None):
        """Resets the environment to an initial state."""
        super().reset(seed=seed)
        
        observation = self._get_obs()

        return observation

    def step(self, action):
        """Executes one time step in the environment."""
        # De-normalize and apply action
        arm_joint_targets = self.robot.initial_joint_pos + action[:7] * 0.1 # Small delta
        gripper_target = 0.04 if action[7] > 0 else 0.0 # Binarize gripper action
        self.robot.set_action(arm_joint_targets, gripper_target)
        
        # Step simulation
        p.stepSimulation(physicsClientId=self.client_id)

        # Get results
        observation = self._get_obs()
        terminated = False
        truncated = False # Can be used for time limits
        reward = 0.0

        return observation, reward, terminated, {}

    def _get_obs(self):
        """Returns the current observation."""
        robot_state = self.robot.get_state()
        img = self.render_gopro(width=self.render_size, width=self.render_size).astype(np.float32)
        img_obs = np.moveaxis(img/255, -1, 0)
        
        return {
            "image": img_obs,
            "hand_pos": robot_state['hand_pos'],
        }

    def _compute_reward(self, obs):
        """Computes the reward for the current state."""
        # Reward is negative distance to the goal
        dist = np.linalg.norm(obs['object_pos'] - obs['socket_pos'])
        return -dist

    def render(self):
        """Renders the environment (only used in 'rgb_array' mode)."""
        if self.render_mode == 'rgb_array':
            view_matrix = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=[0.5, 0, 0.5],
                distance=1.2, yaw=90, pitch=-30, roll=0, upAxisIndex=2)
            proj_matrix = p.computeProjectionMatrixFOV(
                fov=60, aspect=1.0, nearVal=0.1, farVal=100.0)
            
            (_, _, rgba_px, _, _) = p.getCameraImage(
                width=self.render_size, height=self.render_size, viewMatrix=view_matrix,
                projectionMatrix=proj_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
            
            rgb_array = np.array(rgba_px, dtype=np.uint8)
            rgb_array = rgb_array[:, :, :3] # Remove alpha channel
            return rgb_array
        else:
            # 'human' mode is handled by p.GUI
            return None

    def render_gopro(self, width=640, height=480):
        """
        Renders the GoPro view from the robot's perspective.
        
        Returns:
            An RGB image as a NumPy array.
        """
        if hasattr(self, 'robot') and hasattr(self.robot, 'get_camera_image'):
            # Call the get_camera_image method you created earlier
            rgb_array = self.robot.get_camera_image(img_width=width, img_height=height)
            return rgb_array
        else:
            # Return a blank image if the robot or method doesn't exist
            print("Warning: Robot or get_camera_image method not found.")
            return np.zeros((height, width, 3), dtype=np.uint8)

    def close(self):
        """Cleans up the environment."""
        p.disconnect(self.client_id)
