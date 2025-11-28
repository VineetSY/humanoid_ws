#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math

class MotorController(Node):
    def __init__(self):
        super().__init__('motion_generator') # Node name remains same for launch compatibility
        
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.state_sub = self.create_subscription(String, '/robot/system_state', self.state_callback, 10)
        
        self.current_sys_state = "STANDBY"
        self.timer = self.create_timer(0.02, self.control_loop) # 50Hz Control Loop
        self.t = 0.0
        
        # 32 DOF Joint Map
        self.joint_names = [
            'waist_yaw', 'neck_pitch', 'neck_yaw',
            'left_shoulder_roll', 'left_shoulder_pitch', 'left_elbow_pitch', 'left_wrist_roll',
            'right_shoulder_roll', 'right_shoulder_pitch', 'right_elbow_pitch', 'right_wrist_roll',
            'left_hip_pitch', 'left_knee_pitch', 'left_ankle_pitch',
            'right_hip_pitch', 'right_knee_pitch', 'right_ankle_pitch'
        ]
        
        # Current Position (Actual) vs Target Position (Commanded)
        self.current_pos = {name: 0.0 for name in self.joint_names}
        self.target_pos = {name: 0.0 for name in self.joint_names}

    def state_callback(self, msg):
        self.current_sys_state = msg.data

    def smooth_motor_drive(self, joint, target, speed_factor=0.05):
        """
        Simulates Motor Driver Logic: Soft Start / Ramp Up.
        Instead of jumping to target, we move incrementally.
        """
        error = target - self.current_pos[joint]
        # Proportional Control (P-Controller)
        step = error * speed_factor
        self.current_pos[joint] += step

    def set_targets(self):
        # 1. REACH LEFT BEHAVIOR
        if "REACH_LEFT" in self.current_sys_state:
            self.target_pos['left_shoulder_pitch'] = -1.5 # Lift Arm
            self.target_pos['left_elbow_pitch'] = -0.5    # Extend
            self.target_pos['neck_yaw'] = 0.5             # Look Left
            
        # 2. REACH RIGHT BEHAVIOR
        elif "REACH_RIGHT" in self.current_sys_state:
            self.target_pos['right_shoulder_pitch'] = -1.5
            self.target_pos['right_elbow_pitch'] = -0.5
            self.target_pos['neck_yaw'] = -0.5

        # 3. AUTO IDLE BEHAVIOR
        elif self.current_sys_state == "AUTO":
            # Gentle breathing
            self.target_pos['neck_pitch'] = 0.1 * math.sin(self.t * 1.0)
            self.target_pos['left_shoulder_roll'] = 0.1
            self.target_pos['right_shoulder_roll'] = -0.1
            # Reset Arms
            self.target_pos['left_shoulder_pitch'] = 0.0
            self.target_pos['right_shoulder_pitch'] = 0.0

        # 4. FAILSAFE (Limp Home)
        elif self.current_sys_state == "FAILSAFE":
            self.target_pos['left_knee_pitch'] = 1.2
            self.target_pos['right_knee_pitch'] = 1.2
            self.target_pos['left_shoulder_pitch'] = 0.0
            self.target_pos['right_shoulder_pitch'] = 0.0
            
        # 5. STANDBY
        else:
            # Freeze all
            pass

    def control_loop(self):
        self.t += 0.02
        
        # Calculate Targets based on Intelligence/State
        self.set_targets()
        
        # Execute Motor Control (Smooth Movement)
        for j in self.joint_names:
            self.smooth_motor_drive(j, self.target_pos[j])

        # Publish Feedback (Encoder Simulation)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [self.current_pos[n] for n in self.joint_names]
        
        # Simulate Velocity calculation for Load Monitoring
        # (Difference between last pos and curr pos)
        msg.velocity = [0.0] * len(self.joint_names) # Placeholder
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
