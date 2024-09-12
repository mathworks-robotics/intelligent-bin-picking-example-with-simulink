# Intelligent Bin Picking Solutions with Simulink

In robotics, bin picking involves the retrieval of items from a bin using a manipulator. *Intelligent bin picking* enhances this process through increased autonomy, enabling the identification and grasping of parts, followed by the planning of collision-free paths, and then sorting and placing the parts at their designated location based on their classification. This involves the use of sensors, computer vision, and machine learning algorithms to recognize and locate objects within a semi-structured environment, where objects are positioned in a defined space, unaligned and not overlapping, enabling robots to pick specific items with precision. In this intelligent bin picking system, the goal is to identify, classify, and sort four different shapes of PVC fittings.

Explore our specialized collection featuring intelligent bin picking solutions with Simulink®. These examples focus exclusively on intelligent bin picking, involving Universal Robots cobots and photo-realistic 3D simulation environments. Each example provides an in-depth analysis of how technologies such as deep learning, computer vision, and automated motion planning can overcome the challenges of intelligent bin picking in semi-structured environments. Explore these solutions to understand how they integrate various technologies to enhance the efficiency and intelligence of robotic picking systems.

## Examples Overview

### [Example 1: Intelligent Bin Picking with Simulink 3D Animation Unreal Environments (Simulink® and Simulink® 3D Animation™)](example1-sim3D)

**Release:** R2024a

This example demonstrates an intelligent bin picking system for a semi-random object distribution, by using Simulink 3D Animation Unreal Engine for simulation. Task planning, perception, and planning algorithms are defined in Simulink.

**Initial Setup:** 
- None

**Highlights:**
- 6-DoF Pose estimation for object grasp pose identification.
- Simulink 3D Animation Unreal Engine for photo-realistic 3D simulation.
- Manipulator motion planning algorithms for collision-free object handling.

**When to Use:** Use this example to explore bin picking solutions that do not require any third-party tools. It is also a good starting point for those looking to delve into photo-realistic 3D simulation environments and advanced pose estimation techniques.


### [Example 2: Intelligent Bin Picking with Universal Robots UR5e Cobot (Simulink® and Universal Robots UR5e cobot/ Gazebo)](example2-urCobot-gazebo)

**Release:** R2023a

This example demonstrates an intelligent bin picking application for a semi-random object distribution, by using Gazebo for simulation and then performing bin picking by directly connecting to UR Series cobot. Task planning, perception, and planning algorithms are defined in Simulink®. 

**Initial Setup:** 
- You will need a Linux machine that contains Gazebo, ROS, and additional ROS plugins specified [here](https://insidelabs-git.mathworks.com/adityas/intelligent-bin-picking-example-with-simulink/-/tree/main/example2-%20urCobot-gazebo?ref_type=heads#installation). Download a virtual machine from this [page](https://in.mathworks.com/help/ros/ug/get-started-with-gazebo-and-a-simulated-turtlebot.html) to get started.

**Highlights:**
- Seamless Connectivity to Gazebo
- Manipulator motion planning algorithms for collision-free object handling.
- MATLAB/ROS Interface to Universal Robots for hardware and simulation connectivity

**When to Use:**  Use this example when you want to simulate the robot in Gazebo or on a UR Series manipulator. 


## Detailed Comparison of Examples

Here's a quick comparison to help you decide which example is best suited for your needs:

| Feature/Requirement        | Example 1: Intelligent Bin Picking with Simulink 3D Animation Unreal environments | Example 2: Intelligent Bin Picking with Universal Robots UR5e Cobot |
|----------------------------|---------------------------------------|------------------------------|
| **Best for**            | Bin picking using photo-realistic 3D simulation in a game engine environment | Bin picking with ROS and Gazebo, or with ROS and UR series hardware                  |
| **Supported Simulation and Hardware Targets**                | Unreal Engine with Simulink 3D Animation                            | Gazebo & UR Series Manipulator                  |
| **Requires Third-Party Tools**          | No | Requires a Linux machine or virtual machine with Gazebo, ROS, and ROS plug-ins. A pre-configured virtual machine is provided. |
| **GPU Requirement**        | Yes                                | No                       |
| **Tools used for Pose Estimation**        | [Pose Mask R-CNN 6-DoF Pose Estimation](https://in.mathworks.com/help/vision/ug/example-PoseEstimationForBinPickingUsingDeepLearningExample.html)                                | [YOLOv4 with PCA & ICP algorithm](https://in.mathworks.com/help/robotics/urseries/ug/simulink-intelligent-bin-pick-pvc-ur5e-example.html#SimulinkIntelligentBinPickUR5ePVCExample-3)                  |
| **Tools used for Motion Planning**        | [CHOMP planner](https://www.mathworks.com/help/robotics/ref/manipulatorchomp.html) & [TOPP-RA Constrained Trajectory Generator](https://in.mathworks.com/help/robotics/ref/contopptraj.html)                                | [CHOMP planner](https://www.mathworks.com/help/robotics/ref/manipulatorchomp.html)  & [TOPP-RA Constrained Trajectory Generator](https://in.mathworks.com/help/robotics/ref/contopptraj.html)                     |
| **Supported Grippers**        | Vacuum & 2-Finger	                                | Vacuum & 2-Finger	                       |
| **Compatible with**        | R2024a and Beyond                                | R2023a                       |


## License

The license for these examples is available in the License file within each repository.

## Community Support

For questions and support, visit the [MATLAB Central page](https://in.mathworks.com/matlabcentral/fileexchange/128699-intelligent-bin-picking-with-simulink-for-ur5e-cobot) for the support package or post your queries on [MATLAB Answers](https://in.mathworks.com/matlabcentral/answers/index).

Copyright 2024 The MathWorks, Inc.
