classdef GripperTypeEnum < Simulink.IntEnumType
    %TargetEnvs Enum for the targets for pick and place
    %   Targets currently supported are Gazebo, URSim, Hardware
    
    enumeration
        Vaccum(0)
        TwoFinger(1)
    end    

end
