%   Copyright 2023 The MathWorks, Inc.

prompt = {'Enter IP address of ROS enabled device:','Username:','Password:','ROS installation folder:','UR ROS driver installation workspace folder:'};
dlgtitle = 'ROS enabled device details';
dims = [1 50];
if exist('ROSDeviceAddress', 'var')
    ROSDeviceAddressinput = ROSDeviceAddress;
else
    ROSDeviceAddressinput = '0.0.0.0';
end
if exist('username', 'var')
    usernameinput = username;
else
    usernameinput = 'username';
end
if exist('password', 'var')
    passwordinput = password;
else
    passwordinput = 'password';
end
if exist('ROSFolder', 'var')
    ROSFolderinput = ROSFolder;
else
    ROSFolderinput = '/opt/ros/noetic';
end
if exist('WorkSpaceFolder', 'var')
    WorkSpaceFolderinput = WorkSpaceFolder;
else
    WorkSpaceFolderinput = '~/ur_ws';
end
definput ={ROSDeviceAddressinput, usernameinput, passwordinput, ROSFolderinput,WorkSpaceFolderinput };
answer = inputdlg(prompt,dlgtitle,dims,definput);
if ~isempty(answer)
    ROSDeviceAddress = answer{1};
    username = answer{2};
    password = answer{3};
    ROSFolder = answer{4};
    WorkSpaceFolder = answer{5};
else
    ROSDeviceAddress = definput{1};
    username = definput{2};
    password = definput{3};
    ROSFolder = definput{4};
    WorkSpaceFolder = definput{5};
end

set_param([gcs, '/ROSDeviceAddress Display'], ...
    'MaskDisplay', 'color(''red''); try ipad = evalin(''base'', ''ROSDeviceAddress''); catch ipad = ''0.0.0.0''; end; disp(ipad)');
