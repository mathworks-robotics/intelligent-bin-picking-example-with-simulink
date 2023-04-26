if ~(targetEnv == TargetEnvs.Hardware)
    prompt = {'Enter IP address of ROS enabled device:'};
    dlgtitle = 'ROS IP Address';
    dims = [1 50];
    if exist('ROSDeviceAddress', 'var')
        definput = {ROSDeviceAddress};
    else
        definput = {'0.0.0.0'};
    end
    answer = inputdlg(prompt,dlgtitle,dims,definput);
    if ~isempty(answer)
        ROSDeviceAddress = answer{1};
    else
        ROSDeviceAddress = definput{1};
    end
    
    set_param([gcs, '/ROSDeviceAddress Display'], ...
        'MaskDisplay', 'color(''red''); try ipad = evalin(''base'', ''ROSDeviceAddress''); catch ipad = ''0.0.0.0''; end; disp(ipad)');
else
    prompt = {'Enter IP address of ROS enabled device:','Username:','Password:','ROS installation folder:','UR ROS driver installation workspace folder:'};
    dlgtitle = 'ROS enabled device details';
    dims = [1 50];
    if exist('ROSDeviceAddress', 'var')
        ROSDeviceAddressinput = ROSDeviceAddress;
    else
        dROSDeviceAddressinput = '0.0.0.0';
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
        ROSFolderinput = '/opt/ros/melodic';
    end
    if exist('WorkSpaceFolder', 'var')
        WorkSpaceFolderinput = WorkSpaceFolder;
    else
        WorkSpaceFolderinput = '~/ur_ws';
    end
    definput ={dROSDeviceAddressinput, usernameinput, passwordinput, ROSFolderinput,WorkSpaceFolderinput };
    answer = inputdlg(prompt,dlgtitle,dims,definput);
    if ~isempty(answer)
        ROSDeviceAddress = answer{1};
        username = answer{2};
        password = answer{3};
        ROSFolder = answer{4};
        WorkSpaceFolder = answer{5};
    else
        ROSDeviceAddress = definput{1};
    end
    
    set_param([gcs, '/ROSDeviceAddress Display'], ...
        'MaskDisplay', 'color(''red''); try ipad = evalin(''base'', ''ROSDeviceAddress''); catch ipad = ''0.0.0.0''; end; disp(ipad)');
   
end