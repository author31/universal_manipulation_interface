#!/usr/bin/env python3
"""
Interactive Franka Assembly Environment Visualization Tool

This script provides an interactive simulation environment for testing and customizing
the Franka Assembly environment before deploying policies. It includes manual robot control,
real-time parameter adjustment, camera switching, and scene customization.

Usage:
    python visualize.py
    
Controls:
    - WASD + QE: Move robot end-effector
    - Arrow keys: Rotate robot orientation
    - Space: Toggle gripper open/close
    - C: Switch camera views
    - R: Reset robot to home position
    - P: Print current robot state
    - ESC: Exit simulation
"""

import pybullet as p
import numpy as np
import time
import sys
import os
from typing import Dict, Any, Tuple

# Add the project root to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))

from diffusion_policy.env.franka_assembly.franka_assembly_env import FrankaAssemblyEnv
from diffusion_policy.env.franka_assembly.robots.franka import Franka


class FrankaVisualizer:
    """
    Interactive visualization tool for Franka Assembly environment.
    Provides manual control, camera switching, and real-time parameter adjustment.
    """
    
    def __init__(self):
        """Initialize the visualization environment."""
        self.env = None
        self.robot = None
        self.client_id = None
        
        # Control parameters
        self.movement_speed = 0.02
        self.rotation_speed = 0.1
        self.gripper_open = True
        
        # Camera modes
        self.camera_modes = ['follow', 'front', 'side', 'top', 'gopro']
        self.current_camera = 0
        
        # Robot state tracking
        self.target_position = np.array([1, 0.0, 0.7])
        self.target_orientation = np.array([0, 0, 0])
        
        # Control flags
        self.running = True
        self.paused = False
        
        # Performance tracking
        self.last_update_time = time.time()
        self.fps_counter = 0
        self.fps = 0
        
    def initialize_environment(self):
        """Initialize the Franka Assembly environment with GUI."""
        print("🤖 Initializing Franka Assembly Visualization Environment...")
        
        # Create environment with GUI mode
        self.env = FrankaAssemblyEnv(render_mode='human')
        self.client_id = self.env.client_id
        self.robot = self.env.robot
        
        # Set up PyBullet GUI for better interaction
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        
        # Set initial camera
        self.set_camera_view('follow')
        
        # Add debug parameters
        self.add_debug_parameters()
        
        print("✅ Environment initialized successfully!")
        self.print_controls()
        
    def add_debug_parameters(self):
        """Add debug parameters to PyBullet GUI."""
        p.addUserDebugParameter("Movement Speed", 0.001, 0.1, self.movement_speed)
        p.addUserDebugParameter("Rotation Speed", 0.01, 0.5, self.rotation_speed)
        p.addUserDebugParameter("Gripper Force", 10, 200, 100)
        p.addUserDebugParameter("Simulation Speed", 0.1, 2.0, 1.0)
        
    def set_camera_view(self, mode: str):
        """Set camera view based on mode."""
        if mode == 'follow':
            # Follow robot from behind
            p.resetDebugVisualizerCamera(
                cameraDistance=1.5,
                cameraYaw=45,
                cameraPitch=-30,
                cameraTargetPosition=[0.5, 0, 0.5]
            )
        elif mode == 'front':
            # Front view
            p.resetDebugVisualizerCamera(
                cameraDistance=2.0,
                cameraYaw=0,
                cameraPitch=-20,
                cameraTargetPosition=[0.5, 0, 0.5]
            )
        elif mode == 'side':
            # Side view
            p.resetDebugVisualizerCamera(
                cameraDistance=2.0,
                cameraYaw=90,
                cameraPitch=-20,
                cameraTargetPosition=[0.5, 0, 0.5]
            )
        elif mode == 'top':
            # Top view
            p.resetDebugVisualizerCamera(
                cameraDistance=2.0,
                cameraYaw=0,
                cameraPitch=-89,
                cameraTargetPosition=[0.5, 0, 0.5]
            )
        elif mode == 'gopro':
            # GoPro view from robot
            p.resetDebugVisualizerCamera(
                cameraDistance=0.3,
                cameraYaw=0,
                cameraPitch=0,
                cameraTargetPosition=[0.6, 0, 0.7]
            )
            
        print(f"📷 Camera view changed to: {mode}")
        
    def handle_keyboard_input(self):
        """Handle keyboard input for robot control."""
        keys = p.getKeyboardEvents()
        
        # Movement controls (use arrow keys to avoid conflict with PyBullet's built-in keys)
        if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
            self.target_position[1] += self.movement_speed  # Forward
        if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
            self.target_position[1] -= self.movement_speed  # Backward
        if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
            self.target_position[0] -= self.movement_speed  # Left
        if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
            self.target_position[0] += self.movement_speed  # Right
            
        # Up/Down movement
        if ord('q') in keys and keys[ord('q')] & p.KEY_IS_DOWN:
            self.target_position[2] += self.movement_speed  # Up
        if ord('e') in keys and keys[ord('e')] & p.KEY_IS_DOWN:
            self.target_position[2] -= self.movement_speed  # Down
            
        # Rotation controls (use I/J/K/L keys)
        if ord('i') in keys and keys[ord('i')] & p.KEY_IS_DOWN:
            self.target_orientation[2] += self.rotation_speed  # Yaw left
        if ord('j') in keys and keys[ord('j')] & p.KEY_IS_DOWN:
            self.target_orientation[2] -= self.rotation_speed  # Yaw right
        if ord('k') in keys and keys[ord('k')] & p.KEY_IS_DOWN:
            self.target_orientation[1] += self.rotation_speed  # Pitch up
        if ord('l') in keys and keys[ord('l')] & p.KEY_IS_DOWN:
            self.target_orientation[1] -= self.rotation_speed  # Pitch down
            
        # Function keys
        if ord(' ') in keys and keys[ord(' ')] & p.KEY_WAS_TRIGGERED:
            self.toggle_gripper()
        if ord('c') in keys and keys[ord('c')] & p.KEY_WAS_TRIGGERED:
            self.switch_camera()
        if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
            self.reset_robot()
        if ord('p') in keys and keys[ord('p')] & p.KEY_WAS_TRIGGERED:
            self.print_robot_state()
        if ord('h') in keys and keys[ord('h')] & p.KEY_WAS_TRIGGERED:
            self.print_controls()
        if p.B3G_BACKSPACE in keys and keys[p.B3G_BACKSPACE] & p.KEY_WAS_TRIGGERED:
            self.running = False
            
    def toggle_gripper(self):
        """Toggle gripper between open and closed."""
        self.gripper_open = not self.gripper_open
        status = "opened" if self.gripper_open else "closed"
        print(f"🤏 Gripper {status}")
        
    def switch_camera(self):
        """Switch to next camera view."""
        self.current_camera = (self.current_camera + 1) % len(self.camera_modes)
        camera_mode = self.camera_modes[self.current_camera]
        self.set_camera_view(camera_mode)
        
    def reset_robot(self):
        """Reset robot to home position."""
        self.robot.reset()
        self.target_position = np.array([0.5, 0.0, 0.7])
        self.target_orientation = np.array([0, 0, 0])
        self.gripper_open = True
        print("🔄 Robot reset to home position")
        
    def print_robot_state(self):
        """Print current robot state information."""
        state = self.robot.get_state()
        print("\n📊 Current Robot State:")
        print(f"   Hand Position: {state['hand_pos']}")
        print(f"   Hand Orientation: {state['hand_rot']}")
        print(f"   Left Finger: {state['l_finger_pos']:.3f}")
        print(f"   Right Finger: {state['r_finger_pos']:.3f}")
        print(f"   Arm Joints: {state['arm_joint_pos']}")
        print(f"   Target Position: {self.target_position}")
        print(f"   Gripper: {'Open' if self.gripper_open else 'Closed'}")
        print(f"   Camera Mode: {self.camera_modes[self.current_camera]}")
        
    def print_controls(self):
        """Print available controls."""
        print("\n🎮 Available Controls:")
        print("   Arrow Keys: Move robot end-effector (Forward/Back/Left/Right)")
        print("   Q/E: Move robot up/down")
        print("   I/J/K/L: Rotate robot orientation")
        print("   Space: Toggle gripper open/close")
        print("   C: Switch camera view")
        print("   R: Reset robot to home")
        print("   P: Print robot state")
        print("   H: Show this help")
        print("   ESC: Exit simulation")
        print("\n📷 Camera Modes: follow, front, side, top, gopro")
        
    def update_control_parameters(self):
        """Update control parameters from GUI sliders."""
        # Get movement speed from GUI
        movement_speed_param = p.readUserDebugParameter(0)
        if movement_speed_param != self.movement_speed:
            self.movement_speed = movement_speed_param
            
        # Get rotation speed from GUI
        rotation_speed_param = p.readUserDebugParameter(1)
        if rotation_speed_param != self.rotation_speed:
            self.rotation_speed = rotation_speed_param
            
        # Get simulation speed
        sim_speed = p.readUserDebugParameter(3)
        if hasattr(self, '_last_sim_speed') and self._last_sim_speed != sim_speed:
            p.setRealTimeSimulation(sim_speed > 1.0)
            self._last_sim_speed = sim_speed
            
    def update_robot_control(self):
        """Update robot control based on current target position."""
        if self.robot is None:
            return
            
        # Get current robot state
        state = self.robot.get_state()
        current_pos = state['hand_pos']
        
        # Simple position control
        error = self.target_position - current_pos
        
        # Apply action to move towards target
        if np.linalg.norm(error) > 0.001:  # Only move if error is significant
            # Convert position error to joint angles (simplified)
            action = np.zeros(8)
            
            # Simple inverse kinematics approximation
            action[0] = np.clip(error[0] * 2, -1, 1)  # X movement
            action[1] = np.clip(error[1] * 2, -1, 1)  # Y movement  
            action[2] = np.clip(error[2] * 2, -1, 1)  # Z movement
            
            # Add rotation control
            action[3] = np.clip(self.target_orientation[0] * 0.5, -1, 1)
            action[4] = np.clip(self.target_orientation[1] * 0.5, -1, 1)
            action[5] = np.clip(self.target_orientation[2] * 0.5, -1, 1)
            
            # Gripper control
            action[7] = 1.0 if self.gripper_open else -1.0
            
            # Apply action using robot's action method
            arm_targets = self.robot.initial_joint_pos + action[:7] * 0.1
            gripper_target = 0.04 if self.gripper_open else 0.0
            self.robot.set_action(arm_targets, gripper_target)
            
    def update_fps_counter(self):
        """Update FPS counter."""
        current_time = time.time()
        self.fps_counter += 1
        
        if current_time - self.last_update_time >= 1.0:
            self.fps = self.fps_counter
            self.fps_counter = 0
            self.last_update_time = current_time
            
    def render_status_text(self):
        """Render status text in the PyBullet window."""
        status_text = [
            f"FPS: {self.fps}",
            f"Camera: {self.camera_modes[self.current_camera]}",
            f"Gripper: {'Open' if self.gripper_open else 'Closed'}",
            f"Target: ({self.target_position[0]:.2f}, {self.target_position[1]:.2f}, {self.target_position[2]:.2f})",
            "Press H for help"
        ]
        
        # Add text overlay (simplified - PyBullet has limited text overlay support)
        for i, text in enumerate(status_text):
            p.addUserDebugText(
                text,
                [0, 0, 1.0 - i * 0.05],
                textColorRGB=[1, 1, 1],
                textSize=1.2,
                lifeTime=0.1
            )
            
    def run_simulation(self):
        """Main simulation loop."""
        print("🚀 Starting simulation...")
        print("   Press H for help, ESC to exit")
        
        try:
            while self.running:
                self.env.unwrapped.render_gopro(width=1280, height=800)
                # Handle input
                self.handle_keyboard_input()
                
                # Update control parameters
                self.update_control_parameters()
                
                # Update robot control
                self.update_robot_control()
                
                # Step simulation
                p.stepSimulation()
                
                # Update counters
                self.update_fps_counter()
                
                # Render status
                self.render_status_text()
                
                # Control simulation speed
                time.sleep(1/240)  # 240 Hz simulation
                
        except KeyboardInterrupt:
            print("\n⚠️  Simulation interrupted by user")
        except Exception as e:
            print(f"\n❌ Simulation error: {e}")
        finally:
            self.cleanup()
            
    def cleanup(self):
        """Clean up resources."""
        print("🧹 Cleaning up...")
        if self.env:
            self.env.close()
        print("✅ Simulation ended")
        
    def run(self):
        """Main entry point for the visualization tool."""
        try:
            self.initialize_environment()
            self.run_simulation()
        except Exception as e:
            print(f"❌ Failed to start visualization: {e}")
            import traceback
            traceback.print_exc()


def main():
    """Main function for the visualization tool."""
    print("🤖 Franka Assembly Environment Visualization Tool")
    print("=" * 50)
    
    visualizer = FrankaVisualizer()
    visualizer.run()


if __name__ == "__main__":
    main()
