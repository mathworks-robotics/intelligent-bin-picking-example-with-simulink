[detectorFile,detectorPath] = uigetfile('*.mat');
detectorFilePath = [detectorPath,detectorFile];
blkHandle = 'IntelligentBinPicking/Object Detector/Deep Learning Object Detector';
set_param(blkHandle,'DetectorFilePath',detectorFilePath);
blkHandle = 'IntelligentBinPicking/Object Detector/Deep Learning Object Detector';
set_param(blkHandle,'DetectorFilePath',detectorFilePath);