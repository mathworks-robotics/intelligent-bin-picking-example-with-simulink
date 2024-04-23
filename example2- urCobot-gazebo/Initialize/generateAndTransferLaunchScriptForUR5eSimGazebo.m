function generateAndTransferLaunchScriptForUR5eSimGazebo(device,WorkSpaceFolder)
%This function is for internal use only and may be removed in the future.
% generateAndTransferLaunchScriptForUR5eSimGazebo create the launch script to launch
% the required ROS drivers for the UR5e. After that, it copies this script
% to the ROS machine provided and launches the script automatically. 

%Copyright 2023 The MathWorks, Inc.

% Open a file to write a set of commands to launch interface with UR5e
fid=fopen(fullfile(tempdir,"launchURBinPicking.sh"),"w+");
fprintf(fid,"#!/bin/sh\n");
fprintf(fid,"export SVGA_VGPU10=0\n");
fprintf(fid,"export ROS_IP=%s\n",device.DeviceAddress);
fprintf(fid,"export ROS_MASTER_URI=http://$ROS_IP:11311\n");
fprintf(fid,"export IGN_IP=127.0.0.1\n");
fprintf(fid,"export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/user/src/GazeboPlugin/export\n");
fprintf(fid,"gnome-terminal --title=\42UR5 Robot\42 -- /bin/bash -c 'source %s/setup.bash; source %s/devel/setup.bash; roslaunch ur_bin_picking ur_bin_picking.launch'",device.ROSFolder,WorkSpaceFolder);
fclose(fid);

% Copy file into ROS device
putFile(device,fullfile(tempdir,'launchURBinPicking.sh'),'~/')

% Make the shell script executable
system(device,'chmod a+x ~/launchURBinPicking.sh');
end
