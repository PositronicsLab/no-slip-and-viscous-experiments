%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%

% computes the time-derivative of the constraint Jacobians
function [G1c, G2c] = joint_Jacobians_dot( sim, j )

    Jnt = sim.joints(j); 
    body1 = sim.bodies(Jnt.body1id);
    body2 = sim.bodies(Jnt.body2id);

    p1 = Jnt.P1;
    v1 = Jnt.V1; 
    x1 = Jnt.X1 - p1;
    y1 = Jnt.Y1 - p1;
    z1 = Jnt.Z1 - p1;
    vx1 = Jnt.dotX1 - v1;
    vy1 = Jnt.dotY1 - v1;
    vz1 = Jnt.dotZ1 - v1;
    r1 = p1 - body1.u;
    rv1 = v1 - body1.nu;

    p2 = Jnt.P2;
    v2 = Jnt.V2; 
%     x2 = Jnt.X2 - p2;
%     y2 = Jnt.Y2 - p2;
%     z2 = Jnt.Z2 - p2;
    r2 = p2 - body2.u; 
    rv2 = v2 - body2.nu;

    zrs = zeros(3,1); 

    % Not compuationally efficient, but let's write the whole thing
    if ~body1.dynamic
       G1c = zeros(6);
    else
       G1c = [      vx1               vy1             vz1    zrs   zrs   zrs 
              cross3(r1,vx1)+cross(rv1,x1)   cross3(r1,vy1)+cross(rv1,y1)   cross3(r1,vz1)+cross(rv1,z1)   vx1    vy1    vz1  ];
    end
    if ~body2.dynamic
       G2c = zeros(6);
    else
       G2c = [     -vx1              -vy1              -vz1          zrs    zrs    zrs 
              cross3(r2,-vx1)+cross(rv2,-x1)   cross3(r2,-vy1)+cross(rv2,-y1)   cross3(r2,-vz1)+cross(rv2,-z1)   -vx1    -vy1    -vz1  ];
    end

    % Apply mask
    G1c = G1c(:,Jnt.mask);
    G2c = G2c(:,Jnt.mask); 

end


