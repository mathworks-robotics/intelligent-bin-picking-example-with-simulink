function sph=exampleHelperApproximateCollisionBoxSpheresBinPicking(box,rho)
            %exampleHelperApproximateCollisionBoxSpheres Approximate collision box with collection of spheres

            % This function is for internal use only, and maybe removed in the future.

            % Copyright 2022 The MathWorks, Inc.
            %#codegen
            rhoby2=rho/2;
            x=-box.X/2+rhoby2:rho:box.X/2-rhoby2;
            y=-box.Y/2+rhoby2:rho:box.Y/2-rhoby2;
            z=-box.Z/2+rhoby2:rho:box.Z/2-rhoby2;
            if(isempty(x))
                x=box.X;
            end
            if(isempty(y))
                y=box.Y;
            end
            if(isempty(z))
                z=box.Z;
            end
            [x_,y_,z_]=meshgrid(x,y,z);
            points=[x_(:),y_(:),z_(:)];
            centers=transform(se3(box.Pose),points);
            sph=[repmat(rhoby2,1,size(centers,1));centers'];
        end