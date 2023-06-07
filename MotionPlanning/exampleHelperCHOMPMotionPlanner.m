function optimpickconfig = exampleHelperCHOMPMotionPlanner(targetFlag, ...
    binLength, binWidth, binHeight, binCenterPosition, binRotation, ...
    numObstacles, obstaclePose, configs, objYaw)
%exampleHelperCHOMPMotionPlanner Implementation of CHOMP motion planner for
%MEX
% Commands for MEXing this function
% bl = coder.typeof(1);
% bw = coder.typeof(1);
% bh = coder.typeof(1);
% bcp = coder.typeof([1, 1,2]);
% br = coder.typeof(1);
% no = coder.typeof(1);
% tf = coder.typeof(uint8(1));
% cnfgs = coder.typeof(1, [3,6], [1 0]);
% obs = coder.typeof(1, [20, 4], [1, 0]);
% objYaw = coder.typeof(1);
% codegen exampleHelperCHOMPMotionPlanner.m -args {tf, bl, bw, bh, bcp, br, no, obs, cnfgs, objYaw}
%#codegen

%Create a collision environment for the motion planner
env = exampleHelperGenerateCollisionEnviornmentInPlanner(binLength, binWidth, binHeight, binCenterPosition, binRotation, numObstacles, obstaclePose);

%Spherical approximation of the collision environment
sphPerUnit = 22; % Spheres per unit length of box
rho = 1/sphPerUnit;
sphobstacles = zeros(4,500);
coder.varsize('sphobstacles', [4, 500], [0 1]);
startIdx = 1;
for k = 1:length(env)
    sph = exampleHelperApproximateCollisionBoxSpheresBinPicking(env{k},rho);
    sphobstacles(:, startIdx:startIdx+size(sph,2)-1) = sph;
    startIdx = startIdx + 1;
end

timePoints = [0, 2];

%Create persistent variables for the rigidBodyTree and 2 CHOMP planners
persistent robot chompPlace chompPick

if isempty(robot)
    %RBT model of UR5e
    rbt = importrobot("universalUR5e.urdf", 'DataFormat', 'row', 'MaxNumBodies',22);
    
    %Add gripper
    robot = exampleHelperAddGripper(rbt);


    %Modify the transform between th base and the first body for hardware
    if targetFlag == 2 % Hardware/URSim
        tformZYX = eul2tform([0 0 0]);
        setFixedTransform(robot.Base.Children{1,2}.Joint,tformZYX);
    end
end


if isempty(chompPick)
    chompPick = manipulatorCHOMP(robot);
    chompPick.SphericalObstacles = sphobstacles;
    chompPick.CollisionOptions.CollisionCostWeight = 12;
    chompPick.SmoothnessOptions.SmoothnessCostWeight = 1e-5;
    chompPick.SolverOptions.LearningRate = 5;
end

if isempty(chompPlace)
    %For the "place" action, add the object to the end of the RBT for
    %collision checking
    robot = exampleHelperAttachPartAtEndEffector(robot,objYaw);

    chompPlace = manipulatorCHOMP(robot);
    chompPlace.SphericalObstacles = sphobstacles;
    chompPlace.CollisionOptions.CollisionCostWeight = 12;
    chompPlace.SmoothnessOptions.SmoothnessCostWeight = 1e-5;
    chompPlace.SolverOptions.LearningRate = 5;
end

%In this method, if there are 3 sets of configs, it is assumed to be the
%place action and if there are only 2 configs, it is assumed to be the
%pick action.
if size(configs, 1) == 3
    %Timepoints for place action
    timePoints = [0 1 2];

    optimpickconfig = optimize(chompPlace,configs, ... % Starting and ending robot joint configurations
                timePoints, ...                    % Two waypoint times, first at 0s and last at 2s
                0.10, ...                     % 0.1s time step
                InitialTrajectoryFitType="minjerkpolytraj");

else
    optimpickconfig = optimize(chompPick,configs, ... % Starting and ending robot joint configurations
                timePoints, ...                    % Two waypoint times, first at 0s and last at 2s
                0.10, ...                     % 0.1s time step
                InitialTrajectoryFitType="minjerkpolytraj");
end

end



