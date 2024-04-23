%% This script contains the parameters used in the project modules' perception, motion planning, and integration.
% This script runs at the time of project initialization. Change this
% parameters accordingly if you are using any different setup.

%If only one initial detection needs to be performed, set to true
isSingleDetection = true;

% Target environment
targetEnv = TargetEnvs.Gazebo;

%% Perception parameters
% YOLOv4 parameters
yoloScoreThr = 0.8; % YOLO score threshold
gridDownsample = 0.001;% Measured point downsample grid size
PlaneThickness = 0.015;
normalvector = [0 0 1];
maxPlaneTilt = 5; % in degrees



%% Motion planning parameters

% home position
homePositionGZ = deg2rad([-15 -126 113 -80 -91 76]);
homePosition = homePositionGZ;
retractHomePosition= homePosition;

% Place position on IK failure
IKFailurePlaceConfigGZ = [0.7950 -0.5093 0.2500 -1.3115 -1.5708 0];
IKFailurePlaceConfig = IKFailurePlaceConfigGZ;

% Bin and part dimensions
binLength = 0.31; % Along X axis
binWidth = 0.5; % Along Y axis
binHeight = 0.11;
binCenterPosition = [0.46 0 -0.035];
binRotation = 0;

% PVC object dimensions (approximate)
% This will be used for the collision creation
motionParam.partheight = 0.0508;
motionParam.partWidth = 0.0508;
motionParam.partLength = 0.1016;

% Transformation matrix: From camera to the world reference frame
rotationFromRobotToCam = [0.0155 0.9989 -0.0447;0.9997 -0.0147 0.0176; 0.0169 -0.0450 -0.9988];
posDepthCam = [0.5747 -0.0372 0.63];

% Set max acceleration and velocity limits for motion planning with
% hardware

maxqd = pi/1.3; %rad / s
maxqdd = deg2rad(180); % rad / s2

vellimits = repmat([-maxqd; maxqd],1,6);
accellimits  = repmat([-maxqdd; maxqdd],1,6);

% Position threshold for IK
poseThr = 1e-3;

loadPickAndPlaceParametersSL

collisionEnv = {};