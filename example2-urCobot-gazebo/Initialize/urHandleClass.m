classdef urHandleClass < handle
    % This is a helper class to handle the various Serial, UDP and TCP
    % objects using Map (name-value pair arguments)

    %   Copyright 2021 The MathWorks, Inc.

    methods(Static)

        function URNodeMapObj = manageURNodeMap(key,value,action)
            % action - 'add'    : adds the key,value pair to the map
            %          'get'    : returns the object from Map based on key
            %          'remove' : remove object from Map based on key

            % Initialize the persistent variable if it's empty by creating
            % the container Map
            persistent URNodeMap;
            if isempty(URNodeMap)
                URNodeMap = containers.Map;
            end

            URNodeMapObj =[];
            switch action
                case 'add'
                    URNodeMap(key) = value;
                case 'get'
                    if URNodeMap.isKey(key)
                        URNodeMapObj = URNodeMap(key);
                    end
                case 'remove'
                    if URNodeMap.isKey(key)
                        remove(URNodeMap,key);
                    end
            end
        end

    end
end