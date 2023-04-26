# Intelligent Bin Picking with Universal Robots UR5e Cobot for a semi-random object distribution using Simulink
<!-- This is the "Title of the contribution" that was approved during the Community Contribution Review Process --> 

[![View <File Exchange Title> on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/####-file-exchange-title)  
<!-- Add this icon to the README if this repo also appears on File Exchange via the "Connect to GitHub" feature --> 

An Intelligent Bin Picking application consists of a perception algorithm to detect objects and a motion planning algorithm that creates the trajectory path of the robot to pick up the object. First the positions and orientations of the objects lying in the bin are identified from the Camera images using Deep Learning based Image processing capability from Computer Vision Toolbox. The calculated pose is then passed to motion-planning algorithm as an input which plans the path and the trajectory is generated for the Cobot to pick the objects from the bin and place it. 
Using the [Robotics System Toolbox&trade; Support Package for Universal Robots UR Series Manipulators](https://www.mathworks.com/matlabcentral/fileexchange/117530-robotics-system-toolboxtm-support-package-for-universal-robots-ur-series-manipulators), you can establish ROS communication with Universal Robots manipulators and send the generated trajectory for the Cobot to pick up and place the objects at a destination location.This example is a Simulink based solution for the featured examples [Semi-Structured Intelligent Bin Picking for UR5e Using YOLO and PCA-Based Object Detection](https://in.mathworks.com/help/supportpkg/urseries/ug/semi-structured-bin-picking-ur5e-yolo-pca-example.html) and [Gazebo Simulation of Semi-Structured Intelligent Bin Picking for UR5e Using YOLO and PCA-Based Object Detection](https://in.mathworks.com/help/supportpkg/urseries/ug/gazebo-simulation-ur5e-semistructured-intelligent-bin-picking-example.html).

<!--- If your project includes a visualation or any images or an App please include a screenshot in this README --->

<!--- Markdown supports the following HTML entities: © - &copy;  ® - &reg;  ™ - &trade;
More information about Trademarks can be found internally within the Checklist for Community Contributions and Supportfiles Confluence page--->

<!--- Please remember to delete all template related text that you are not using within your README.md ---> 

### MathWorks Products (https://www.mathworks.com)

Requires MATLAB&reg; release R2023a or higher
- [MATLAB&reg;](https://www.mathworks.com/products/matlab.html)
- [Simulink&reg;](https://in.mathworks.com/products/simulink.html)
- [Robotics System Toolbox&trade;](https://www.mathworks.com/products/robotics.html)
- [ROS Toolbox](https://www.mathworks.com/products/ros.html)
- [Computer Vision Toolbox&trade;](https://www.mathworks.com/products/computer-vision.html)
- [Image Processing Toolbox&trade;](https://www.mathworks.com/products/image.html)
- [Deep Learning Toolbox&trade;](https://www.mathworks.com/products/deep-learning.html)
- [Optimization Toolbox&trade;](https://www.mathworks.com/products/optimization.html)
- [Statistics and Machine Learning Toolbox&trade;](https://www.mathworks.com/products/statistics.html)
- [Robotics System Toolbox Support Package for Universal Robots UR Series Manipulators](https://www.mathworks.com/matlabcentral/fileexchange/117530-robotics-system-toolbox-support-package-for-universal-robots-ur-series-manipulators)

Optional Products
- [MATLAB Coder&trade;](https://www.mathworks.com/products/matlab-coder.html)
- [Computer Vision Toolbox Model for YOLO v4 Object Detection](https://www.mathworks.com/matlabcentral/fileexchange/107969-computer-vision-toolbox-model-for-yolo-v4-object-detection)

## Installation
Installation instructions

1. MATLAB installation: Visit installation instructions [webpage](https://in.mathworks.com/help/install/) to get started with the MATLAB installation process. 
2. Support package installation: Before proceeding, ensure that the products mentioned under MathWorks Products above are installed. To install the Robotics System Toolbox Support Package for Universal Robots UR Series Manipulators, follow the steps mentioned [here](https://in.mathworks.com/help/supportpkg/urseries/ug/install-support-for-manipulator-hardware.html).  
3. Download ROS Virtual Machine For Pre-Installed Gazebo Environment: You can download a virtual machine image that already has ROS and Gazebo installed. This virtual machine is based on Ubuntu® Linux® and is pre-configured to support the examples in ROS Toolbox™. Download and install the ROS virtual machine from this [link](https://in.mathworks.com/help/ros/ug/get-started-with-gazebo-and-a-simulated-turtlebot.html).
4. YOLO Object Detector MAT files and Dataset: The example uses a pre-trained YOLOV4 object detector for identifying the Cuboid objects. Downlaod the detector MAt file from the following links.
- [Object detector and Dataset for Gazebo Simulation](https://ssd.mathworks.com/supportfiles/UniversalRobots/IntelligentBinPickingDataSet/Cuboidal_Object_Dataset_Sim.zip)
- [Object detector and Dataset for UR5e Hardware](https://ssd.mathworks.com/supportfiles/UniversalRobots/IntelligentBinPickingDataSet/Cuboidal_Object_Dataset_Real.zip)

## Getting started with Bin picking example
Open the the project to get started with bin picking example.The MLX file associated have the necessory background and steps to complate the example.

open('IntelligentBinPickingExampleWithSimulink.prj')

## Basic Examples

To learn how to communicate with UR Series cobots over ROS, see [Getting Started with Connecting and Controlling a UR5e Cobot from Universal Robots](https://www.mathworks.com/help/supportpkg/urseries/ug/getting-started-controlling-ur5e.html). 
<!--- Make sure you have a repo set up correctly if you are to follow this formatting --->

## License
<!--- Make sure you have a License.txt within your Repo --->
The license is available in the License file within this repository.


## Community Support
You can post your queries on the [MATLAB Central](https://in.mathworks.com/matlabcentral/fileexchange/117530-robotics-system-toolboxtm-support-package-for-universal-robots-ur-series-manipulators) page for the support package.
You can also add your questions at [MATLAB Answers](https://www.mathworks.com/matlabcentral/answers/index).


Copyright 2023 The MathWorks, Inc.

<!--- Do not forget to the add the SECURITY.md to this repo --->
<!--- Add Topics #Topics to your Repo such as #MATLAB  --->

<!--- This is my comment --->

<!-- Include any Trademarks if this is the first time mentioning trademarked products (For Example:  MATLAB&reg; Simulink&reg; Trademark&trade; Simulink Test&#8482;) --> 

