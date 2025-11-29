#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math

class MotorController(Node):
    def __init__(self):
        super().__init__('motion_generator')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.state_sub = self.create_subscription(String, '/robot/system_state', self.state_callback, 10)
        self.current_sys_state = "STANDBY"
        self.timer = self.create_timer(0.02, self.control_loop)
        self.t = 0.0
        
        self.joint_names = [
            'virtual_root_z', 'waist_yaw', 'neck_pitch', 'neck_yaw',
            'left_shoulder_roll', 'left_shoulder_pitch', 'left_elbow_pitch', 'left_wrist_roll',
            'right_shoulder_roll', 'right_shoulder_pitch', 'right_elbow_pitch', 'right_wrist_roll',
            'left_hip_pitch', 'left_knee_pitch', 'left_ankle_pitch',
            'right_hip_pitch', 'right_knee_pitch', 'right_ankle_pitch'
        ]
        
        self.current_pos = {name: 0.0 for name in self.joint_names}
        self.target_pos = {name: 0.0 for name in self.joint_names}

    def state_callback(self, msg):
        self.current_sys_state = msg.data

    def smooth_motor_drive(self, joint, target, speed_factor=0.05):
        error = target - self.current_pos[joint]
        step = error * speed_factor
        self.current_pos[joint] += step

    def set_targets(self):
        target_height = 0.0
        
        # === NEW: REACHING = SQUATTING ===
        if "REACH_LEFT" in self.current_sys_state:
            # 1. Squat Down
            target_height = -0.35
            self.target_pos['left_knee_pitch'] = 1.8
            self.target_pos['right_knee_pitch'] = 1.8
            self.target_pos['left_hip_pitch'] = -0.9
            self.target_pos['right_hip_pitch'] = -0.9
            self.target_pos['left_ankle_pitch'] = -0.9
            self.target_pos['right_ankle_pitch'] = -0.9

            # 2. Reach Arm Down towards floor
            self.target_pos['left_shoulder_pitch'] = 0.5  # Swing arm forward/down
            self.target_pos['left_elbow_pitch'] = -0.2    # Almost straight arm
            self.target_pos['neck_pitch'] = 0.3           # Look at floor
            self.target_pos['waist_yaw'] = 0.3            # Twist torso
            
        elif "REACH_RIGHT" in self.current_sys_state:
            # Mirror of Left
            target_height = -0.35
            self.target_pos['left_knee_pitch'] = 1.8; self.target_pos['right_knee_pitch'] = 1.8
            self.target_pos['left_hip_pitch'] = -0.9; self.target_pos['right_hip_pitch'] = -0.9
            self.target_pos['left_ankle_pitch'] = -0.9; self.target_pos['right_ankle_pitch'] = -0.9
            self.target_pos['right_shoulder_pitch'] = 0.5
            self.target_pos['right_elbow_pitch'] = -0.2
            self.target_pos['neck_pitch'] = 0.3
            self.target_pos['waist_yaw'] = -0.3

        # === NEW: LIFTING = STANDING UP ===
        elif self.current_sys_state == "LIFTING":
            # 1. Stand Up (Height 0)
            target_height = 0.0
            
            # 2. Reset Legs
            self.target_pos['left_knee_pitch'] = 0.0; self.target_pos['right_knee_pitch'] = 0.0
            self.target_pos['left_hip_pitch'] = 0.0; self.target_pos['right_hip_pitch'] = 0.0
            self.target_pos['left_ankle_pitch'] = 0.0; self.target_pos['right_ankle_pitch'] = 0.0

            # 3. Curl Arms (Holding Box)
            self.target_pos['left_shoulder_pitch'] = -0.5
            self.target_pos['left_elbow_pitch'] = -1.5  
            self.target_pos['right_shoulder_pitch'] = -0.5
            self.target_pos['right_elbow_pitch'] = -1.5 
            self.target_pos['neck_pitch'] = 0.0   
            self.target_pos['waist_yaw'] = 0.0      

        elif self.current_sys_state == "AUTO":
            # Idle
            self.target_pos['neck_pitch'] = 0.1 * math.sin(self.t * 1.0)
            self.target_pos['left_shoulder_roll'] = 0.1; self.target_pos['right_shoulder_roll'] = -0.1
            
            # Full Reset
            self.target_pos['left_shoulder_pitch'] = 0.0; self.target_pos['right_shoulder_pitch'] = 0.0
            self.target_pos['left_elbow_pitch'] = 0.0; self.target_pos['right_elbow_pitch'] = 0.0
            self.target_pos['neck_yaw'] = 0.0
            self.target_pos['left_hip_pitch'] = 0.0; self.target_pos['left_knee_pitch'] = 0.0; self.target_pos['left_ankle_pitch'] = 0.0
            self.target_pos['right_hip_pitch'] = 0.0; self.target_pos['right_knee_pitch'] = 0.0; self.target_pos['right_ankle_pitch'] = 0.0

        elif self.current_sys_state == "FAILSAFE":
            target_height = -0.35 
            self.target_pos['left_knee_pitch'] = 1.8; self.target_pos['right_knee_pitch'] = 1.8
            self.target_pos['left_hip_pitch'] = -0.9; self.target_pos['right_hip_pitch'] = -0.9
            self.target_pos['left_ankle_pitch'] = -0.9; self.target_pos['right_ankle_pitch'] = -0.9
            self.target_pos['left_shoulder_pitch'] = 0.5; self.target_pos['right_shoulder_pitch'] = 0.5
            self.target_pos['left_elbow_pitch'] = -2.0; self.target_pos['right_elbow_pitch'] = -2.0
            
        self.target_pos['virtual_root_z'] = target_height

    def control_loop(self):
        self.t += 0.02
        self.set_targets()
        for j in self.joint_names:
            self.smooth_motor_drive(j, self.target_pos[j])
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [self.current_pos[n] for n in self.joint_names]
        msg.velocity = [0.0] * len(self.joint_names)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
