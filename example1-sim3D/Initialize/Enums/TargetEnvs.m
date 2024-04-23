classdef TargetEnvs < Simulink.IntEnumType
    %TargetEnvs Enum for the targets for pick and place
    %   Targets currently supported are Gazebo, URSim, Hardware
    
    enumeration
        Gazebo(0)
        URSim(1)
        Hardware(2)
        Sim3d(3)
    end    

end

