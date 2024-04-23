function [refPose,partID] = exampleHelperSelectPartForMEXPlanner(partGT,binCenterPosition)
%This function is for internal use only and may be removed in the future.
% exampleHelperSelectPartForMEXPlanner selects the  the closest object
% to the center of the bin for the requested pick and place cycle and it gives
% reference pose and part index as an output.

%Copyright 2023 The MathWorks, Inc.

% Select the part which is closest to the center
% Find the euclidean distance from bin center
dist = zeros(1,size(partGT,1));
for i=1:size(partGT,1)
    dist(i) = pdist([partGT(i,1:2);binCenterPosition(1:2)]);
end

% Find the index of the part which is closest to the bin center point 
[~,I] = min(dist);

% Assign the output
refPose = zeros(1, 3);
refPose = [partGT(I,1) partGT(I,2) partGT(I,3)];
partID = I;
end

