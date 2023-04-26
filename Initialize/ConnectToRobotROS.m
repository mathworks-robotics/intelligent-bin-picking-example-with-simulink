% Load Rigid Body Tree model
% Load rigid body tree model for Universal Robot UR5e robot manipulator with 
% an end-effector body. The function |exampleHelperAddGripper| is used to add 
% the additional end-effector body (the Robotiq Epick suction gripper) as a collision 
% object for precise motion planning.

rbt = importrobot('universalUR5e.urdf','MaxNumBodies',22,'DataFormat','row');
ur5e = exampleHelperAddGripper(rbt);

%Hardware specific paramters
rotationFromRobotToCam = [0.0155 0.9989 -0.0447;0.9997 -0.0147 0.0176; 0.0169 -0.0450 -0.9988];
posDepthCam = [0.5747 -0.0372 0.63];
isROSMachineSSHEnbled = true;
% Set Target environment
targetEnv = TargetEnvs.Hardware;

% Place config on IK failure
IKFailurePlaceConfig = IKFailurePlaceConfigGZ + [pi 0 0 0 0 0];
retractHomePosition = [165 -100 103 -95 -90 76]*pi/180;

% Connect to ROS host
% Provide parameters of the host machine with ROS. 
deviceAddress = ROSDeviceAddress; % IP address

if (isROSMachineSSHEnbled)    
    robotIP = robotAddress; % Robot IP address
    
    device = rosdevice(deviceAddress,username,password);
    device.ROSFolder = ROSFolder;
    % Generate launch script and transfer it to ROS host computer.
    % This section creates and launches the script automatically in the ROS host 
    % machine to launch the required ROS nodes for the UR5e robot.
    
    generateAndTransferLaunchScriptForUR5e(device,WorkSpaceFolder,robotIP);
    
    if ~isCoreRunning(device)
        w = strsplit(system(device,'who'));
        displayNum = cell2mat(w(2));
    
        system(device,['export SVGA_VGPU10=0; ' ...
            'export DISPLAY=' displayNum '.0; ' ...
            './launchURBinPicking.sh &']);
        pause(10);
    end
end

% Create an instance of |universalrobot| class 
% The |universalrobot| object has several useful properties. The |RigidBodyTree| 
% property contains the rigid body tree model, which is being used internally 
% for trajectory generation and motion planning. 

% Create an instance to communicate with the UR5e cobot
ur = universalrobot(deviceAddress,'RigidBodyTree',ur5e);
% Move the robot to the home position
% |homePosition| parameter is defined in the |initializeParametersForBinPickingHardware.m| 
% script. You can find this script inside the *Initialize* folder of the project.

% Send To home position
%Add pi to the Base angle of the homePosition
homePosition = homePositionGZ + [pi 0 0 0 0 0];
sendJointConfigurationAndWait(ur,homePosition,'EndTime',5);
clear ur

% Generate and transfer launch script for the IntelRealSense D415 camera sensor
% This section automatically generates, copies, and runs the launch script into 
% the ROS host machine to create ROS interface for the IntelRealSense D415 camera. 
% 
% Follow this document <add link and create page> and run the steps mentioned 
% to install the required ROS drivers for the intelRealSense cameras and setup 
% the connection.
if (isROSMachineSSHEnbled)
    WorkSpaceFolderForCamera = '~/catkin_ws';
    generateAndTransferLaunchScriptForCamera(device,WorkSpaceFolderForCamera);
    
    w = strsplit(system(device,'who'));
    displayNum = cell2mat(w(2));
    
    system(device,['export SVGA_VGPU10=0; ' ...
        'export DISPLAY=' displayNum '.0; ' ...
        './launchURBinPickingCamera.sh &']);
    pause(4);
end

rosshutdown;
ROS_Machine_IP = deviceAddress;
rosinit(ROS_Machine_IP);
