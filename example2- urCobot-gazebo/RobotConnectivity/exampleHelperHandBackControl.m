function exampleHelperHandBackControl()
% Hand Back the control to PolyScope
client = rossvcclient('/ur_hardware_interface/hand_back_control');
call(client);
pause(0.5);
end