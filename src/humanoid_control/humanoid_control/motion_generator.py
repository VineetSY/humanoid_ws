#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math

class MotionGenerator(Node):
    def __init__(self):
        super().__init__('motion_generator')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.state_sub = self.create_subscription(String, '/robot/system_state', self.state_callback, 10)
        self.current_sys_state = "STANDBY"
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.t = 0.0
        
        # 32 DOF Joint Map
        self.joint_names = [
            'waist_yaw', 'neck_pitch', 'neck_yaw',
            'left_shoulder_roll', 'left_shoulder_pitch', 'left_elbow_pitch', 'left_wrist_roll',
            'right_shoulder_roll', 'right_shoulder_pitch', 'right_elbow_pitch', 'right_wrist_roll',
            'left_hip_pitch', 'left_knee_pitch', 'left_ankle_pitch',
            'right_hip_pitch', 'right_knee_pitch', 'right_ankle_pitch'
        ]
        self.joint_positions = {name: 0.0 for name in self.joint_names}

    def state_callback(self, msg):
        self.current_sys_state = msg.data

    def get_auto_pattern(self, t):
        self.joint_positions['neck_yaw'] = 0.3 * math.sin(t * 0.5)
        self.joint_positions['left_shoulder_roll'] = 0.2
        self.joint_positions['right_shoulder_roll'] = -0.2
        self.joint_positions['right_elbow_pitch'] = -1.5 + 0.5 * math.sin(t * 2.0)
        self.joint_positions['right_shoulder_pitch'] = 0.5 * math.cos(t * 1.0)

    def get_limp_home_pattern(self):
        target_knee = 1.0
        current = self.joint_positions['left_knee_pitch']
        if current < target_knee:
            new_pos = current + 0.05
            self.joint_positions['left_knee_pitch'] = new_pos
            self.joint_positions['right_knee_pitch'] = new_pos
        self.joint_positions['left_shoulder_pitch'] = 0.0
        self.joint_positions['right_shoulder_pitch'] = 0.0

    def timer_callback(self):
        self.t += 0.05
        if self.current_sys_state == "AUTO":
            self.get_auto_pattern(self.t)
        elif self.current_sys_state == "FAILSAFE":
            self.get_limp_home_pattern()
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [self.joint_positions[n] for n in self.joint_names]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotionGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
