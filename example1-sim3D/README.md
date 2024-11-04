# Intelligent Bin Picking with Simulink® 3D Animation™ for Semi-Random Object Distribution Using Simulink&reg;
<!-- This is the "Title of the contribution" that was approved during the Community Contribution Review Process --> 

[![View <File Exchange Title> on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/128699-intelligent-bin-picking-with-simulink-for-ur5e-cobot)  
<!-- Add this icon to the README if this repo also appears on File Exchange via the "Connect to GitHub" feature --> 

In robotics, bin picking involves using a manipulator to retrieve items from a bin. Intelligent bin picking represents an advanced version of this process, offering greater autonomy. It possesses the capability to perceive parts, accurately identify and classify them. Then plan collision-free paths to sort the parts based on their classification. 

First, the positions and orientations of the objects lying in the bin are identified from the camera images using pre-trained Pose Mask R-CNN network capability from Computer Vision Toolbox. The calculated pose is then passed as input to motion-planning algorithm, designed using the [manipulatorCHOMP planner](https://www.mathworks.com/help/robotics/ref/manipulatorchomp.html), which plans the path and generates the trajectory for the cobot to pick the objects from the bin and place them. 
Using the [Robotics System Toolbox&trade; Support Package for Universal Robots UR Series Manipulators](https://www.mathworks.com/matlabcentral/fileexchange/117530-robotics-system-toolboxtm-support-package-for-universal-robots-ur-series-manipulators), you can implement a Universal Robots cobot in a 3D environment and send the generated trajectory for the cobot to pick up and place the objects at a destination location. This example covers "Simulink 3D Animation" based solution for the featured examples [Simulink Based Intelligent Bin Picking Using Universal Robots UR5e for PVC Fittings](https://in.mathworks.com/help/robotics/urseries/ug/simulink-intelligent-bin-pick-pvc-ur5e-example.html) and [Perform 6-DoF Pose Estimation for Bin Picking Using Deep Learning.
](https://in.mathworks.com/help/vision/ug/example-PoseEstimationForBinPickingUsingDeepLearningExample.html)

This project demonstrates the integration of a pre-trained Pose Mask R-CNN network for object detection with the Simulink&reg; 3D environment for simulating semi-structured PVC distribution systems. By leveraging the capabilities of the Simulation 3D Actor block and the [Pose Mask R-CNN 6-DoF Object Pose Estimation model](https://in.mathworks.com/matlabcentral/fileexchange/155869-computer-vision-toolbox-model-for-pose-mask-r-cnn-6-dof-object-pose-estimation) from the Computer Vision Toolbox™, you can simulate a photo-realistic environment and detect objects within the 3D environment without the need for external simulation platforms like Gazebo. This integration facilitates rapid design iterations and testing within a familiar Simulink virtual environment.

  
![IBPExample](IBPExample.gif)


<!--- If your project includes a visualation or any images or an App please include a screenshot in this README --->

<!--- Markdown supports the following HTML entities: © - &copy;  ® - &reg;  ™ - &trade;
More information about Trademarks can be found internally within the Checklist for Community Contributions and Supportfiles Confluence page--->

<!--- Please remember to delete all template related text that you are not using within your README.md ---> 

### MathWorks Products (https://www.mathworks.com)

Requires MATLAB&reg; release R2024a or higher
- [MATLAB&reg;](https://www.mathworks.com/products/matlab.html)
- [Simulink&reg;](https://in.mathworks.com/products/simulink.html)
- [MATLAB Coder&trade;](https://www.mathworks.com/products/matlab-coder.html)
- [Parallel Computing Toolbox&trade;](https://in.mathworks.com/products/parallel-computing.html)
- [Stateflow&reg;](https://in.mathworks.com/products/stateflow.html)
- [Robotics System Toolbox&trade;](https://www.mathworks.com/products/robotics.html)
- [Simulink 3D Animation&trade;](https://in.mathworks.com/products/3d-animation.html)
- [Computer Vision Toolbox&trade;](https://www.mathworks.com/products/computer-vision.html)
- [Computer Vision Toolbox Model for Pose Mask R-CNN 6-DoF Object Pose Estimation](https://in.mathworks.com/matlabcentral/fileexchange/155869-computer-vision-toolbox-model-for-pose-mask-r-cnn-6-dof-object-pose-estimation)
- [Image Processing Toolbox&trade;](https://www.mathworks.com/products/image.html)
- [Deep Learning Toolbox&trade;](https://www.mathworks.com/products/deep-learning.html)
- [Optimization Toolbox&trade;](https://www.mathworks.com/products/optimization.html)
- [Statistics and Machine Learning Toolbox&trade;](https://www.mathworks.com/products/statistics.html)
- [Robotics System Toolbox Support Package for Universal Robots UR Series Manipulators](https://www.mathworks.com/matlabcentral/fileexchange/117530-robotics-system-toolbox-support-package-for-universal-robots-ur-series-manipulators)


## Installation
Installation instructions

1. **MATLAB installation**: Visit installation instructions [webpage](https://in.mathworks.com/help/install/) to get started with the MATLAB installation process. Ensure that the products mentioned under MathWorks Products above are installed.
2. **Support package installation**:  To install the Robotics System Toolbox Support Package for Universal Robots UR Series Manipulators, follow the steps mentioned [here](https://in.mathworks.com/help/supportpkg/urseries/ug/install-support-for-manipulator-hardware.html).  
3. **Pose Mask R-CNN 6-DoF Pose Estimation network**: Install the Computer Vision Toolbox Model for Pose Mask R-CNN 6-DoF Object Pose Estimation from Add-On Explorer. 

For more information about installing add-ons, see [Get and Manage Add-Ons](https://in.mathworks.com/help/matlab/matlab_env/get-add-ons.html). The Computer Vision Toolbox Model for Pose Mask R-CNN 6-DoF Object Pose Estimation requires Deep Learning Toolbox™ and Image Processing Toolbox™.

## Getting started with Bin picking example
Open the project to get started with bin picking example. Then, navigate to the `SimulinkModel` directory and open `IntelligentBinPicking.slx`.

```matlab
>> open('IntelligentBinPickingExampleWithSimulink.prj');
>> open_system('SimulinkModel/IntelligentBinPicking.slx');
```
After you start the simulation of the model, open Diagnostic Viewer (click "View Diagnostics") to observe the information about the robot action that is in progress.<br>

## Run Model with Faster Motion Planning
You can reduce the time taken by the motion planning algorithm by creating a MEX function. Generating a MEX function using C/C++ code generation helps to reduce the computation time and hence reduces the pick and place cycle time.<br><br>
For more information on how to create a MEX function for the manipulatorRRT algorithm-based planner, see the [Generate Code for Manipulator Motion Planning in Perceived Environment](https://mathworks.com/help/robotics/ug/generate-code-for-manipulator-motion-planning-in-perceived-environment.html) example.<br><br>
For more information on generating MEX function to accelerate your MATLAB program execution, see the [Accelerate MATLAB Algorithm by Generating MEX Function](https://mathworks.com/help/coder/gs/generating-mex-functions-from-matlab-code-at-the-command-line.html) example.<br><br>

Step1: Create MEX for the exampleHelperCHOMPMotionPlanner function.
```matlab
>> generateMEXForPlanner
```
Step2: Select the `Enable MEX` option in MotionPlannerCHOMP block in the Motion Planner subsystem.<br>

<img src="https://github.com/Aditya-Innovacious/intelligent-bin-picking-example-with-simulink/assets/167069380/f15188e1-5c1d-4466-9ab3-f2b9cacf6ba9" width="600"><br>

## Examples

To learn how to generate scenes for 3D simulation environment, see [Simulation 3D Scene Configuration.
](https://in.mathworks.com/help/driving/ref/simulation3dsceneconfiguration.html) 
  
To learn more about the Pose Mask R-CNN network, see [Perform 6-DoF Pose Estimation for Bin Picking Using Deep Learning.](https://in.mathworks.com/help/vision/ug/example-PoseEstimationForBinPickingUsingDeepLearningExample.html)
  
<!--- Make sure you have a repo set up correctly if you are to follow this formatting --->

## Unreal Engine Simulation Environment Requirements and Limitations

The Unreal Environment has been tested with the specifications listed below. For optimal performance, ensure that your system meets these hardware and software requirements. 

For a detailed list of requirements and limitations, refer to [Unreal Engine Simulation Environment Requirements and Limitations.](https://in.mathworks.com/help/sl3d/unreal-engine-simulation-environment-requirements-and-limitations.html#responsive_offcanvas)

### Tested GPU Configuration

The example has been tested on the following GPU configuration:

#### Windows
```plaintext
GPU configuration:

  Name: 'NVIDIA GeForce RTX 3080'
  Index: 1
  ComputeCapability: '8.6'
  SupportsDouble: 1
  GraphicsDriverVersion: '551.23'
  DriverModel: 'WDDM'
  ToolkitVersion: 12.2000
  MaxThreadsPerBlock: 1024
  MaxShmemPerBlock: 49152 (49.15 KB)
  MaxThreadBlockSize: [1024 1024 64]
  MaxGridSize: [2.1475e+09 65535 65535]
  SIMDWidth: 32
  TotalMemory: 10736893952 (10.74 GB)
  AvailableMemory: 9427219968 (9.43 GB)
  CachePolicy: 'balanced'
  MultiprocessorCount: 68
  ClockRateKHz: 1725000
  ComputeMode: 'Default'
  GPUOverlapsTransfers: 1
  KernelExecutionTimeout: 1
  CanMapHostMemory: 1
  DeviceSupported: 1
  DeviceAvailable: 1
  DeviceSelected: 1
```

#### Debian® 11
```plaintext
GPU configuration:

  Name: 'NVIDIA TITAN Xp'
  Index: 1
  ComputeCapability: '6.1'
  SupportsDouble: 1
  GraphicsDriverVersion: '555.42.02'
  DriverModel: 'N/A'
  ToolkitVersion: 12.2000
  MaxThreadsPerBlock: 1024
  MaxShmemPerBlock: 49152 (49.15 KB)
  MaxThreadBlockSize: [1024 1024 64]
  MaxGridSize: [2.1475e+09 65535 65535]
  SIMDWidth: 32
  TotalMemory: 12774408192 (12.77 GB)
  AvailableMemory: 7546579440 (7.55 GB)
  CachePolicy: 'balanced'
  MultiprocessorCount: 30
  ClockRateKHz: 1582000
  ComputeMode: 'Default'
  GPUOverlapsTransfers: 1
  KernelExecutionTimeout: 1
  CanMapHostMemory: 1
  DeviceSupported: 1
  DeviceAvailable: 1
  DeviceSelected: 1
```

## License
<!--- Make sure you have a License.txt within your Repo --->
The license is available in the License file within this repository.

## Troubleshooting
 
#### Issue: Building MEX function results in this error: `Error(s) encountered while building simulation target MEX-file for model 'IntelligentBinPicking'.`
 
**Solution:** Ensure that the default MEX compilers for both C and C++ code generation are of the same type (for example, MinGW64 Compiler (C) and MinGW64 Compiler (C++)). To check the default MEX compiler for C and C++, run `mex -setup c` and `mex -setup cpp` respectively. MATLAB displays information about the default compilers for each language. If the compiler type is different, click the link of the compiler in the displayed message so that both compiler types match.

#### Issue: Robot joints goes to undesired position multiple times during the simulation.

**Solution:** Change the configuration to send trajectory waypoints at a reduced rate. To do this, perform one of these actions:
- After opening the model `IntelligentBinPicking.slx`, run these two commands in MATLAB to assign new values to the corresponding parameters:
```matlab
>> graspRegionTranslation = [0.0140, 0.0140, 0.0260];
>> downSampleForTrajectoryWaypoints = 10;
```
- Open the initialization script `~\example1-sim3D\Initialize\initRobotModelParam.m`, and modify the two parameters `graspRegionTranslation` and `downSampleForTrajectoryWaypoints` to assign the new values as shown in the first option.

#### Issue: A warning "Known issues with graphic driver" appears, with suggestion to upgrade the graphics driver.

**Solution:** Ideally, you should upgrade to the latest version of the graphics driver. However, if you do not want to upgrade to the latest version, open `IntelligentBinPicking/Object Detector/PoseMaskRCNNModel` block in the Simulink model, and set the value of `ExecutionEnvironment` parameter to `cpu`.

## Community Support
You can post your queries on the [MATLAB Central](https://in.mathworks.com/matlabcentral/fileexchange/117530-robotics-system-toolboxtm-support-package-for-universal-robots-ur-series-manipulators) page for the support package.
You can also add your questions at [MATLAB Answers](https://www.mathworks.com/matlabcentral/answers/index).


Copyright 2024 The MathWorks, Inc.

<!--- Do not forget to the add the SECURITY.md to this repo --->
<!--- Add Topics #Topics to your Repo such as #MATLAB  --->

<!--- This is my comment --->

<!-- Include any Trademarks if this is the first time mentioning trademarked products (For Example:  MATLAB&reg; Simulink&reg; Trademark&trade; Simulink Test&#8482;) --> 
