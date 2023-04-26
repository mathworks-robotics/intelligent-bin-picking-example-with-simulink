% Script to 
% 1. Copy launch file to Gazebo VM
% 2. Launch file
% 3. Connect MATLAB to ROS Master
% 4. Create universalrobot object and sedn robot to user defined home position
% 5. Setup environment objects in Gazebo
% 6. Initialize collision environemnt and set dropping poses for objects

%   Copyright 2023 The MathWorks, Inc.

%% Connect to ROS host
deviceAddress = ROSDeviceAddress; % IP address
username = 'user'; % User name
password = 'password'; % Password corresponds to user name

ROSFolder = '/opt/ros/noetic'; % ROS installation folder
WorkSpaceFolder = '~/ur_ws'; % UR ROS driver installation workspace folder location

device = rosdevice(deviceAddress,username,password);
device.ROSFolder = ROSFolder;

%% Generate launch script and transfer it to ROS host computer.
generateAndTransferLaunchScriptForUR5eSimGazebo(device,WorkSpaceFolder);

if ~isCoreRunning(device)
    w = strsplit(system(device,'who'));
    displayNum = cell2mat(w(2));

    system(device,['export SVGA_VGPU10=0; ' ...
        'export DISPLAY=' displayNum '.0; ' ...
        './launchURBinPicking.sh &']);
    pause(15);
end
gzinit(deviceAddress,14581);
rosshutdown;
rosinit(deviceAddress);

%% Load Rigid Body Tree model and add gripper
if exist('ur5e', 'var') ~= 1
    rbt = loadrobot('universalUR5e','DataFormat','row');
    ur5e = exampleHelperAddGripper(rbt);
end

%% Create an instance to communicate with the simulated UR5e cobot
ur = universalrobot(deviceAddress,'RigidBodyTree',ur5e);
urHandleClass.manageURNodeMap(deviceAddress,[],'remove');
urHandleClass.manageURNodeMap(deviceAddress,ur,'add');

%% Move the robot to the home position
sendJointConfigurationAndWait(ur,homePosition,'EndTime',2);
pause(2);
clear ur

%% Spawn the objects
% call ROS service for pause the Gazebo world
physicsClientPause = rossvcclient('gazebo/pause_physics');
call(physicsClientPause,'Timeout',3);

% Set states for table, bin and stand
gzmodel('set','bin_picking_stand','Position',[0 0 0.001],'Orientation',eul2quat([0,0,0]))
pause(0.5);
gzmodel('set','pvc_tray','Position',[0.48 0 0.55],'Orientation',eul2quat([1.57 0 0]),"SelfCollide","off","IsStatic","on")
pause(0.5);
gzlink('set','table_solid','link','Position',[0.17 0.68 0.4],'Orientation',eul2quat([0,0,0]))

% call ROS service for un-pause the Gazebo world
physicsClientUnpause = rossvcclient('gazebo/unpause_physics');
call(physicsClientUnpause,'Timeout',3);

% create mesh
px1 = -0.20:0.12:0.24;
py1 = -0.12:0.11:0.12;

py1 = py1(randperm(length(py1)))+0.48;
px1 = px1(randperm(length(px1)));

temp = cell(length(px1),length(py1));

for i = 1:length(px1)
    for j = 1:length(py1)
        temp(i,j) = {[px1(i), py1(j)]};
    end
end

newPose = reshape(temp,[1,length(px1)*length(py1)]);

% get the list of the world models
modelList = gzmodel("list");

% list of the target objects (naming starts with part1, part2 etc.)
partList = modelList(contains(modelList,"part"));


ii=randperm(length(newPose));
b=newPose(ii);
partGT = zeros(length(partList), 4);
% spawn the world
for i=1:length(partList)
    partGT(i, 1:2) = cell2mat(b(i));
    partGT(i, 3) = 0.8;
    partGT(i, 4) = rand*pi*0.8;
    gzlink('set',partList(i),'link','Position',[partGT(i, 2) partGT(i, 1) 0.8],'Orientation',eul2quat([partGT(i, 4),0,0]));
end

pause(3);

for i=1:length(partList)
% Check if the part is not in the favourable orientation. Change the
% orientation in this case
    partOrient = quat2eul(gzlink('get',partList(i),'link','Orientation'),"XYZ");
    if abs(partOrient(1)) > 0.01 || abs(partOrient(2)) > 0.01
        gzlink('set',partList(i),'link','Orientation',eul2quat([rand*pi*0.8,0,0]));
    end
end
pause(3);

% Create a collision environment for the planner
collisionEnv = {};

homePose = ur5e.getTransform(homePosition, 'Bellow', 'base');
