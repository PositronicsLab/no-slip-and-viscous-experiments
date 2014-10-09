function [sim, ke] = grasp( nboxes, maxIter )

    % the kinetic energy of the box with respect to the grippers 
    ke = [];

    %% A user function for controlling the wheel torque to achieve a certain speed.
    % userFunctions get called at the beginning of every timestep.  
    function sim = applyForceFunction( sim )
        F = 100*nboxes;
        kv = 0;
        avelD = 1;

        % compute predynamics
        sim = preDynamics(sim);

        % apply forces pushing in
        fx = [F 0 0]; 
        sim.bodies(1).Fext(1:3) = qtrotate(sim.bodies(1).quat,-fx');          
        sim.bodies(2).Fext(1:3) = qtrotate(sim.bodies(1).quat,fx');          

        % compute the metric for each block
        newke = 0;
        dt = sim.h;
        for i=1:length(sim.bodies)-2

          % get the inertia of the block
          M = [eye(3)*sim.bodies(i).mass zeros(3); zeros(3) sim.bodies(i).J];

          % get the velocity of the two grippers
          dot1 = sim.bodies(1).nu(1:6);
          dot2 = sim.bodies(2).nu(1:6);

          % get the position of the block
          x = sim.bodies(2+i).u;

          % get the orientation of the block
          q = sim.bodies(i).quat;

          % set xstar
          R = qt2rot(q);
          xstar1 = sim.bodies(1).u - R*w(1:3,i);
          xstar2 = sim.bodies(2).u - R*y(1:3,i);

          % set dotstar and dotthetastar
          dotxstar1 = sim.bodies(1).nu(1:3);
          dotxstar2 = sim.bodies(2).nu(1:3);
          dotthetastar1 = sim.bodies(1).nu(4:6);
          dotthetastar2 = sim.bodies(2).nu(4:6);

          % get the orientation differential - this assumes initial grasp
          % occurs with both bodies in their identity orientation
          thetadiff1 = qdot2avel(q, sim.bodies(1).quat - q); 
          thetadiff2 = qdot2avel(q, sim.bodies(2).quat - q); 

          % get the relative velocities of the box with respect to both grippers
          v1a = (xstar1 - x)/dt + (dotxstar1 - dot1(1:3));
          v1b = (xstar2 - x)/dt + (dotxstar2 - dot1(1:3));
          v2a = thetadiff1/dt + (dotthetastar1 - dot1(4:6));
          v2b = thetadiff2/dt + (dotthetastar2 - dot2(4:6));
          vx = [v1a; v2a];
          vy = [v1b; v2b];
          newke = newke + 0.5*(0.5*dot(vx, M*vx) + 0.5*dot(vy, M*vy));
        end 

        % update the metric 
        ke = [ke; newke];
    end


    %% Our main function that creates a chassis with four wheels 
    sim = Simulator(.001);
    sim.MAX_STEP = maxIter;
    sim.userFunction = @applyForceFunction;                 % Tell sim to use our userFunction
    sim.H_dynamics = @LCPdynamics;
%    sim.H_dynamics = @NoSlip;  % comment this line out to use S-T again
    sim.H_solver = @Lemke;
    sim.draw = false;
    %sim.drawContacts = true;
    sim.FRICTION = true; 
    sim.num_fricdirs = 4; 
    sim.drawJoints = false; 

    % object 
    for i=1:nboxes
        boxes(i) = mesh_rectangularBlock(.1, .1, .1);
        boxes(i).u = [-nboxes/2+(i-1)+0.5; 0; 0]*.1;
        boxes(i).color = [.3 .6 .5];
        boxes(i).mu = 10^20;
    end

    % apply a random velocity to the box
    box.nu = randn(6,1);
        
    % setup manipulator 
    gripper1 = mesh_cube();  
    gripper2 = mesh_cube();
    gripper1.u = [-nboxes/2+(nboxes-1)+1.5; 0; 0]*.1;
    gripper2.u = [-nboxes/2-0.5; 0; 0]*.1;
    gripper1.mu = 100;
    gripper2.mu = 100;  
    gripper1.color = [1 0 0];
    gripper2.color = [1 0 0];
    w1.u = [ 1.5;  0; 0]*.1;  
    w2.u = [-1.5;  0; 0]*.1; 
    
    % Add bodies to simulator
    sim = sim_addBody(sim, [gripper1 gripper2 boxes]);

    % get the u vectors
    for i=1:nboxes
      w(:,i) = gripper1.u - boxes(i).u;
      y(:,i) = gripper2.u - boxes(i).u;
    end
    
    % Create joints
     sim = sim_addJoint( sim, 1, 2, w1.u, [1;0;0], 'prismatic');
     sim = sim_addJoint( sim, 1, 3, w2.u, [1;0;0], 'prismatic');
    
    % Run the simulator
%    sim = sim_run_Drumwright( sim );
    sim = sim_run( sim );

end

