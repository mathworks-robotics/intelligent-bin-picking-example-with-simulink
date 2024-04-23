function generateAndTransferLaunchScriptForUR5e(device,WorkSpaceFolder,robotIP)
%This function is for internal use only and may be removed in the future.
% generateAndTransferLaunchScriptForUR5e create the launch script to launch
% the required ROS drivers for the UR5e. After that, it copies this script
% to the ROS machine provided and launches the script automatically.

%Copyright 2023 The MathWorks, Inc.

% Open a file to write set of commands to launch interface with UR5e
fid=fopen(fullfile(tempdir,"launchURBinPicking.sh"),"w+");
fprintf(fid,"#!/bin/sh\n");
fprintf(fid,"export SVGA_VGPU10=0\n");
fprintf(fid,"export ROS_IP=%s\n",device.DeviceAddress);
fprintf(fid,"export ROS_MASTER_URI=http://$ROS_IP:11311\n");
fprintf(fid,"gnome-terminal --title=\42UR5 Robot\42 -- /bin/bash -c 'source %s/setup.bash; source %s/devel/setup.bash; roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=%s'",device.ROSFolder,WorkSpaceFolder,robotIP);
fclose(fid);

% Copy file into ROS device
putFile(device,fullfile(tempdir,'launchURBinPicking.sh'),'~/')

% Make the shell script executable
system(device,'chmod a+x ~/launchURBinPicking.sh');
end
