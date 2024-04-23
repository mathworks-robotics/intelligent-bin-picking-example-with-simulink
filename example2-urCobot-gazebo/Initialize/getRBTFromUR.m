function tree = getRBTFromUR(ipaddress)
%GETRBTFROMUR Summary of this function goes here
%   Detailed explanation goes here
ur = urHandleClass.manageURNodeMap(ipaddress,[],'get');
if ~isempty(ur)
    tree = ur.RigidBodyTree;
else
    rbt = loadrobot('universalUR5e','DataFormat','row');
    ur5e = exampleHelperAddGripper(rbt);
    ur = universalrobot(ipaddress,'RigidBodyTree',ur5e);
    tree = ur.RigidBodyTree;
    urHandleClass.manageURNodeMap(ipaddress,ur,'add');
end
end

