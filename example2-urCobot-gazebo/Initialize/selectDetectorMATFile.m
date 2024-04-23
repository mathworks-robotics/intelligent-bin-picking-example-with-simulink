[detectorFile,detectorPath] = uigetfile('*.mat');
detectorFilePath = [detectorPath,detectorFile];
blkHandle = 'IntelligentBinPicking/Object Detector/Object Detector/Gazebo/Deep Learning Object Detector';
set_param(blkHandle,'DetectorFilePath',detectorFilePath);
blkHandle = 'IntelligentBinPicking/Object Detector/Object Detector/URSim_Hardware/Deep Learning Object Detector';
set_param(blkHandle,'DetectorFilePath',detectorFilePath);