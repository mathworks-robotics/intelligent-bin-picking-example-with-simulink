function env = exampleHelperGenerateCollisionEnviornmentInPlanner(binLength, binWidth, binHeight, binCenterPosition, binRotation,numObstacles, obstacles)
        %This function is for internal use only and may be removed in the future.
        % This helper function generates a collision environment array with
        % the bin, parts and the camera

        %   Copyright 2023 The MathWorks, Inc.

        % Add CollisionBox face 1 (Thickness 0.001 assumed)
        b1 = collisionBox(binLength,0.001,binHeight);
        b1.Pose(1:3,1:3) = eul2rotm([0 binRotation*pi/180 0]);
        b1.Pose(1:3,4) = binCenterPosition+[0 binWidth/2 0];

        % Add CollisionBox face 2 (Thickness 0.001 assumed)
        b2 = collisionBox(binLength,0.001,binHeight);
        b2.Pose(1:3,1:3) = eul2rotm([0 binRotation*pi/180 0]);
        b2.Pose(1:3,4) = binCenterPosition+[0 -binWidth/2 0];

        % Add CollisionBox face 3 (Thickness 0.001 assumed)
        b3 = collisionBox(0.001,binWidth,binHeight);
        b3.Pose(1:3,1:3) = eul2rotm([0 binRotation*pi/180 0]);
        b3.Pose(1:3,4) = binCenterPosition + (eul2rotm([0 binRotation*pi/180 0])*[binLength/2;0;0])';

        % Add CollisionBox face 4 (Thickness 0.001 assumed)
        b4 = collisionBox(0.001,binWidth,binHeight);
        b4.Pose(1:3,1:3) = eul2rotm([0 binRotation*pi/180 0]);
        b4.Pose(1:3,4) = binCenterPosition + (eul2rotm([0 binRotation*pi/180 0])*[-binLength/2;0;0])';

        % Add CollisionBox face 5 (Thickness 0.001 assumed)
        b5 = collisionBox(binLength,binWidth,0.001);
        b5.Pose(1:3,1:3) = eul2rotm([0 binRotation*pi/180 0]);
        b5.Pose(1:3,4) = binCenterPosition + (eul2rotm([0 binRotation*pi/180 0])*[0;0;-binHeight/2])';

        % Add CollisionBox Place Table (Thickness 0.05 assumed)
        table = collisionBox(0.5,0.9,0.05);
        table.Pose(1:3,1:3) = eul2rotm([0 0 0]);
        table.Pose(1:3,4) = [0 0.9 -0.09];

        % Add camera as a collision box
        camera = collisionBox(0.04,0.1,0.04);
        camera.Pose(1:3,1:3) = eul2rotm([0 0 0]);
        camera.Pose(1:3,4) = [0.54 0 0.61];

        env = {b1,b2,b3,b4,b5,table,camera}; % Collision enviorment array

        % Define Part dimensions (cuboid)
        partLength = 0.1016;
        partWidth = 0.0508;
        partheight = 0.0508;

        % Create collision from given part ground truth of a box shape
        % using collisionBox
        for i=coder.unroll(1:20)
            if i<=numObstacles
                coder.inline('never');
                B = collisionBox(partLength,partWidth,partheight);
                objPos = obstacles(i, 1:3);
                objYaw = obstacles(i, 4);
                B.Pose(1:3,4) = [objPos(1) objPos(2) (binCenterPosition(3) - (binHeight/2) + (partheight/2) + 0.002)]';
                B.Pose(1:3,1:3) = eul2rotm([deg2rad(objYaw+90) 0 0]);
                env{end+1} = B;
            end

        end
    end