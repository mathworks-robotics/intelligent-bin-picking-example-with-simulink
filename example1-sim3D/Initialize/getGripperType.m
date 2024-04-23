
gripperlist = {'Vacuum','TwoFinger'};
[gripperIndx,isGripperSelected] = listdlg('ListString',gripperlist,'ListSize',[220,60],'SelectionMode','single', ...
    'PromptString','Select the Gripper Type','Name','Gripper Type');
if isGripperSelected
    gripperType = GripperTypeEnum(gripperlist{gripperIndx});
else
    error("Gripper Type not selected. Click on 'Setup for Hardware' button and choose a Gripper Type");
end
