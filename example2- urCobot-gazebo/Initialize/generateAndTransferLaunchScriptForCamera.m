function generateAndTransferLaunchScriptForCamera(device,WorkSpaceFolder)
%This function is for internal use only and may be removed in the future.
% generateAndTransferLaunchScriptForCamera create the launch script to launch
% the required ROS drivers for the IntelRealSense D415. After that, it copies this script
% to the ROS machine provided and launches the script automatically.

%Copyright 2023 The MathWorks, Inc.

%Copyright 2023 The MathWorks, Inc.
fid=fopen(fullfile(tempdir,"launchURBinPickingCamera.sh"),"w+");
fprintf(fid,"#!/bin/sh\n");
fprintf(fid,"export SVGA_VGPU10=0\n");
fprintf(fid,"export ROS_IP=%s\n",device.DeviceAddress);
fprintf(fid,"export ROS_MASTER_URI=http://$ROS_IP:11311\n");

fprintf(fid,"gnome-terminal --title=\42Intel RealSense Camera\42 -- /bin/bash -c 'source %s/setup.bash; source %s/devel/setup.bash; roslaunch realsense2_camera demo_pointcloud.launch'",device.ROSFolder,WorkSpaceFolder);

fclose(fid);

% Copy file into ROS device
putFile(device,fullfile(tempdir,'launchURBinPickingCamera.sh'),'~/')

% Make the shell script executable
system(device,'chmod a+x ~/launchURBinPickingCamera.sh');
end