%% This script contains the parameters used in the project modules' perception, motion planning, and integration.
% This script runs at the time of project initialization. Change this
% parameters accordingly if you are using any different setup.

%If only one initial detection needs to be performed, set to true
isSingleDetection = true;

% Target environment
targetEnv = TargetEnvs.Gazebo;

%% Perception parameters
% YOLOv4 parameters
yoloScoreTh = 0.9;% YOLO score threshold. if 0, all detection are accepted
gridDownsample = 0.001;% Measured point downsample grid size
PlaneThickness = 0.01;
normalvector = [0 0 1];
maxPlaneTilt = 2; % in degrees


%% Motion planning parameters

% home position
homePositionGZ = deg2rad([-15 -126 113 -80 -91 76]);
homePosition = homePositionGZ;
retractHomePosition= homePosition;

% Place position on IK failure
IKFailurePlaceConfigGZ = [0.7950 -0.5093 0.2500 -1.3115 -1.5708 0];
IKFailurePlaceConfig = IKFailurePlaceConfigGZ;

% Bin and part dimensions
binLength = 0.395; % Along X axis
binWidth = 0.585; % Along Y axis
binHeight = 0.11;
binCenterPosition = [0.48 0 -0.038];
binRotation = 0;

% cuboid object dimensions
partheight = 0.0508;
partWidth = 0.0508;

% Transformation matrix: From camera to the world reference frame
rotationFromRobotToCam = eul2rotm([pi/2, pi, 0],"ZYZ");
posDepthCam = [0.53+0.02 0.02 1.2-0.625];

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