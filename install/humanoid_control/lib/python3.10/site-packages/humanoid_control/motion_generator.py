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
        self.timer = self.create_timer(0.01, self.control_loop)
        self.t = 0.0
        
        self.joint_names = [
            'virtual_root_x', 'virtual_root_y', 'virtual_root_z', 
            'waist_yaw', 'neck_pitch', 'neck_yaw',
            'left_shoulder_roll', 'left_shoulder_pitch', 'left_elbow_pitch', 'left_wrist_roll',
            'right_shoulder_roll', 'right_shoulder_pitch', 'right_elbow_pitch', 'right_wrist_roll',
            'left_hip_pitch', 'left_knee_pitch', 'left_ankle_pitch',
            'right_hip_pitch', 'right_knee_pitch', 'right_ankle_pitch'
        ]
        
        self.current_pos = {name: 0.0 for name in self.joint_names}
        self.target_pos = {name: 0.0 for name in self.joint_names}
        self.is_walking = False
        self.walk_phase = 0.0

    def state_callback(self, msg):
        self.current_sys_state = msg.data

    def compute_s_curve(self, current, target, max_step=0.01):
        diff = target - current
        if abs(diff) < 0.001: return target
        step = max(0.002, min(abs(diff) * 0.05, max_step))
        return current + step if diff > 0 else current - step

    def walk_cycle(self):
        self.walk_phase += 0.2
        self.current_pos['left_hip_pitch'] = 0.3 * math.sin(self.walk_phase)
        self.current_pos['right_hip_pitch'] = 0.3 * math.sin(self.walk_phase + math.pi)
        self.current_pos['virtual_root_z'] = 0.02 * math.sin(self.walk_phase * 2)

    def set_targets(self):
        # OBJECT LOCATIONS
        BOX_X = 1.5
        BOX_Y = 0.5
        
        # === FIX: STOPPING DISTANCE ===
        # We stop 0.4 meters BEFORE the box so we can reach it.
        STANCE_X = BOX_X - 0.4 
        STANCE_Y = BOX_Y
        
        HOME_X = 0.0
        HOME_Y = 0.0

        goal_x = self.current_pos['virtual_root_x']
        goal_y = self.current_pos['virtual_root_y']
        needs_to_walk = False

        if "REACH_LEFT" in self.current_sys_state:
            # NAVIGATE TO STANCE (Not Box)
            goal_x = STANCE_X
            goal_y = STANCE_Y
            
            dist = math.sqrt((goal_x - self.current_pos['virtual_root_x'])**2 + (goal_y - self.current_pos['virtual_root_y'])**2)
            
            if dist > 0.05:
                needs_to_walk = True
            else:
                # ARRIVED: EXECUTE PICKUP
                # Squat
                self.target_pos['virtual_root_z'] = -0.35
                self.target_pos['left_knee_pitch'] = 1.8; self.target_pos['right_knee_pitch'] = 1.8
                self.target_pos['left_hip_pitch'] = -0.9; self.target_pos['right_hip_pitch'] = -0.9
                self.target_pos['left_ankle_pitch'] = -0.9; self.target_pos['right_ankle_pitch'] = -0.9
                
                # Reach FORWARD (X axis) to bridge the gap
                self.target_pos['left_shoulder_pitch'] = -0.8 # Arm forward
                self.target_pos['left_elbow_pitch'] = -0.5    # Slight bend
                self.target_pos['waist_yaw'] = 0.0            # Face forward

        elif self.current_sys_state == "LIFTING":
            # Stand up at location
            self.target_pos['virtual_root_z'] = 0.0
            self.target_pos['left_knee_pitch'] = 0.0; self.target_pos['right_knee_pitch'] = 0.0
            self.target_pos['left_hip_pitch'] = 0.0; self.target_pos['right_hip_pitch'] = 0.0
            self.target_pos['left_ankle_pitch'] = 0.0; self.target_pos['right_ankle_pitch'] = 0.0
            
            # Curl Arms
            self.target_pos['left_shoulder_pitch'] = -0.5; self.target_pos['left_elbow_pitch'] = -1.5
            self.target_pos['right_shoulder_pitch'] = -0.5; self.target_pos['right_elbow_pitch'] = -1.5

        elif self.current_sys_state == "AUTO":
            goal_x = HOME_X
            goal_y = HOME_Y
            dist = math.sqrt((goal_x - self.current_pos['virtual_root_x'])**2 + (goal_y - self.current_pos['virtual_root_y'])**2)
            if dist > 0.05:
                needs_to_walk = True
            else:
                # Reset
                self.target_pos['left_shoulder_pitch'] = 0.0; self.target_pos['right_shoulder_pitch'] = 0.0
                self.target_pos['left_elbow_pitch'] = 0.0; self.target_pos['right_elbow_pitch'] = 0.0
                self.target_pos['virtual_root_z'] = 0.0
        
        elif self.current_sys_state == "FAILSAFE":
             self.target_pos['virtual_root_z'] = -0.35 
             self.target_pos['left_knee_pitch'] = 1.8; self.target_pos['right_knee_pitch'] = 1.8
             self.target_pos['left_hip_pitch'] = -0.9; self.target_pos['right_hip_pitch'] = -0.9
             self.target_pos['left_ankle_pitch'] = -0.9; self.target_pos['right_ankle_pitch'] = -0.9
             self.target_pos['left_shoulder_pitch'] = 0.5; self.target_pos['right_shoulder_pitch'] = 0.5
             self.target_pos['left_elbow_pitch'] = -2.0; self.target_pos['right_elbow_pitch'] = -2.0

        self.target_pos['virtual_root_x'] = goal_x
        self.target_pos['virtual_root_y'] = goal_y
        self.is_walking = needs_to_walk

    def control_loop(self):
        self.t += 0.01
        self.set_targets()
        
        self.current_pos['virtual_root_x'] = self.compute_s_curve(self.current_pos['virtual_root_x'], self.target_pos['virtual_root_x'], 0.02)
        self.current_pos['virtual_root_y'] = self.compute_s_curve(self.current_pos['virtual_root_y'], self.target_pos['virtual_root_y'], 0.02)
        
        if not self.is_walking:
            self.current_pos['virtual_root_z'] = self.compute_s_curve(self.current_pos['virtual_root_z'], self.target_pos['virtual_root_z'], 0.02)

        for j in self.joint_names:
            if "virtual" not in j:
                if not self.is_walking:
                    self.current_pos[j] = self.compute_s_curve(self.current_pos[j], self.target_pos[j])
        
        if self.is_walking:
            self.walk_cycle()

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
