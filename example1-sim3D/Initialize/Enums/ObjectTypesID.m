classdef ObjectTypesID < Simulink.IntEnumType
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    enumeration
        Invalid(-1)
        % Make sure to start with one because we use the enum to index
        % MATLAB arrays
        Unknown  (0)
        I        (1)
        L        (2)
        T        (3)
        X        (4)
    end    

end