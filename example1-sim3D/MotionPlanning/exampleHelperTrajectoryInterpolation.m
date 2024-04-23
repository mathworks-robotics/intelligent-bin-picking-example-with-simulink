function [q_interp, qd_interp, qdd_interp, tSample] = exampleHelperTrajectoryInterpolation(pieceWisePolynomial,trajectorySampleTime,vellimits,accellimits)
%This function is for internal use only and may be removed in the future.
% exampleHelperTrajectoryInterpolation computes the trajectory and it's
% interpolation for the smooth motion which also satisfies the given velocity
% acceleration limits using contopptraj and interp1

%Copyright 2023 The MathWorks, Inc.

% Find joint's position, velocity, acceleration, and time array using
% contopptraj within the desired acceleration and velocity limits
[q,qd,qdd,t] = contopptraj(pieceWisePolynomial,vellimits,accellimits,NumSamples=50);

% Interpolate the trajectory for a given sample time 
tSample = t(1):trajectorySampleTime:t(end);
q_interp = interp1(t,q',tSample,'pchip');
qd_interp = interp1(t,qd',tSample,'pchip');
qdd_interp = interp1(t,qdd',tSample,'pchip');

end