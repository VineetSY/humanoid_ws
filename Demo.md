# **32-DoF Humanoid: Mobility & Manipulation Controller**

**Hackathon Demo Walkthrough**

## **1\. Project Overview**

This project demonstrates a comprehensive ROS2 control architecture for a 32-Degree-of-Freedom humanoid robot. The system features autonomous path planning, kinematic manipulation, active load management, and safety failsafes.

### **Achieved Goals**

| Task | Feature Implemented |
| :---- | :---- |
| **Control System** | Centralized system\_supervisor node managing state logic. |
| **Motor Management** | Smooth S-Curve trajectory generation to minimize motor jerk. |
| **Kinematics & Planning** | Autonomous navigation to targets and Inverse Kinematics (IK) approximation for grasping. |
| **Load Management** | Active monitoring of gripper load; successfully lifts \<20kg, rejects \>20kg. |
| **Failsafe Design** | "Limp Home" crouch protocol triggered by thermal or load anomalies. |
| **Remote Operations** | Service-based handover between "Standby" (Teleop ready) and "Autonomous" modes. |

## **2\. Startup Sequence**

*Perform this before the judges arrive.*  
**Terminal 1: The Simulation Core**  
cd \~/humanoid\_ws  
source install/setup.bash  
ros2 launch humanoid\_control hackathon\_bringup.launch.py

* **Visual Check:** RViz should open. The robot is at (0,0) in "STANDBY" mode (Frozen). A red target box is visible at (1.5, 0.5).

Terminal 2: The Trigger Console  
Prepare this terminal for sensor inputs.  
source \~/humanoid\_ws/install/setup.bash

## **3\. Demo Script (The Presentation)**

### **Phase 1: Remote Operations & Handover**

*Goal: Demonstrate the robot starting up and accepting control.*

1. **Narrator:** "Currently, the robot is in a safe **Standby Mode**. We will now authorize the handover to Autonomous Control."  
2. **Action (Terminal 2):**  
   ros2 service call /control/request\_teleop std\_srvs/srv/SetBool "{data: false}"

3. **Visual Result:** The robot begins its "Idle Breathing" animation.  
4. **Narrator:** "The System Supervisor has accepted the handover. The robot is now live."

### **Phase 2: Path Planning & Kinematics (The Success Story)**

*Goal: Demonstrate navigation, path planning, and successful manipulation.*

1. **Narrator:** "Our Perception Unit has identified a target object. The robot calculates a path to approach it safely."  
2. **Action (Terminal 2 \- Trigger Perception):**  
   ros2 topic pub /perception/trigger\_event std\_msgs/msg/String "{data: 'TARGET\_LEFT'}" \--once

3. **Visual Result:**  
   * **Navigation:** Robot calculates vector to (1.5, 0.5).  
   * **Locomotion:** Robot slides across the floor with leg-swing animation.  
   * **Arrival:** Robot stops **40cm short** of the box (Collision Avoidance).  
   * **Manipulation:** Robot squats down, extends arm, and opens gripper.  
4. **Narrator:** "The robot has arrived and entered the 'Reach' posture. We now simulate a successful grasp of a 10kg payload."  
5. **Action (Terminal 2 \- Trigger Safe Load):**  
   ros2 topic pub /sensors/grip\_load std\_msgs/msg/Float32 "{data: 10.0}" \--once

6. **Visual Result:**  
   * **Grasping:** The red box snaps to the robot's hand.  
   * **Lifting:** The robot stands up, curling the object to its chest.  
7. **Narrator:** "Load accepted. The robot will now return to base."  
8. **Action (Terminal 2 \- Reset):**  
   ros2 service call /control/request\_teleop std\_srvs/srv/SetBool "{data: true}"

9. **Visual Result:** Robot walks back to (0,0) carrying the box.

### **Phase 3: Failsafe & Safety Systems (The Failure Story)**

*Goal: Demonstrate load management and emergency protocols.*

1. **Narrator:** "Now we test the safety limits. We will attempt to lift an overload scenario (25kg)."  
2. **Action (Terminal 2 \- Trigger Perception Again):**  
   ros2 topic pub /perception/trigger\_event std\_msgs/msg/String "{data: 'TARGET\_LEFT'}" \--once

3. **Visual Result:** Robot walks to the target and reaches down again.  
4. **Action (Terminal 2 \- Trigger OVERLOAD):**  
   ros2 topic pub /sensors/grip\_load std\_msgs/msg/Float32 "{data: 25.0}" \--once

5. **Visual Result:**  
   * **Safety Trigger:** Console logs CRITICAL FAILURE: OVERLOAD.  
   * **Reaction:** Robot immediately drops the box (box falls to floor).  
   * **Failsafe:** Robot enters "Safety Crouch" (fetal position) to lower center of gravity and protect motors.  
6. **Narrator:** "The system detected a 25kg load, exceeding the 20kg safety limit. It immediately jettisoned the payload and entered 'Limp Home' mode to prevent structural damage."

## **4\. Troubleshooting Cheat Sheet**

**Problem: Robot isn't moving.**

* Fix: Did you run the handover command? The robot starts in Standby.  
  ros2 service call /control/request\_teleop std\_srvs/srv/SetBool "{data: false}"

**Problem: Box isn't appearing.**

* *Fix:* In RViz, ensure the **Marker** display is added and listening to /simulation/target\_object.

**Problem: Robot is stuck in Failsafe.**

* *Fix:* The Failsafe is a "latching" state (for safety). You must restart the simulation (Ctrl+C in Terminal 1\) to reset.