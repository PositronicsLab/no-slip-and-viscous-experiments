function [sim, ke, cv] = grasp( )

    % the kinetic energy of the box with respect to the grippers 
    ke = [];
    cv = [];

    %% A user function for controlling the wheel torque to achieve a certain speed.
    % userFunctions get called at the beginning of every timestep.  
    function sim = applyForceFunction( sim )
        F = 10;

        % apply forces pushing in
        fx = [F 0 0]; 
        sim.bodies(3).Fext(1:3) = -fx';          
        sim.bodies(4).Fext(1:3) = fx';          
        sim.bodies(5).Fext(1:3) = -fx';          
        sim.bodies(6).Fext(1:3) = fx';          
        sim.bodies(7).Fext(1:3) = -fx';          
        sim.bodies(8).Fext(1:3) = fx';          
        sim.bodies(9).Fext(1:3) = -fx';          
        sim.bodies(10).Fext(1:3) = fx';          

        % get the relative velocities of the box with respect to both grippers
        rv1 = sim.bodies(2).nu - sim.bodies(3).nu;
        rv2 = sim.bodies(2).nu - sim.bodies(4).nu;

        sim = DrumwrightPredynamics(sim);

        % update the ke
        ke = [ke; 0.25*(dot(rv1,rv1) + dot(rv2,rv2))];
        cv = [cv; min(sim.dynamics.Gn'*sim.dynamics.NU)];
    end


    %% Our main function that creates a chassis with four wheels 
    sim = Simulator(.001);
    sim.MAX_STEP = 1000;
    sim.userFunction = @applyForceFunction;                 % Tell sim to use our userFunction
    sim.H_dynamics = @Drumwrightdynamics;
    %sim.drawContacts = true;
    sim.FRICTION = true; 
    sim.num_fricdirs = 4; 
    sim.drawJoints = false; 

    % ground
    ground = Body_plane([0; 0; -1.5], [0;0;1]);
      ground.color = [0 0 0];
      ground.dynamic = false;
      ground.visible = false;
    
    % object 
    box = mesh_rectangularBlock(1, 1, 1);
        box.u = [0; 0; 0];
        box.color = [.3 .6 .5];
        box.mu = 1;

    % setup manipulators 
    gripper1 = mesh_rectangularBlock(1, 0.5, 0.5);  
    gripper2 = mesh_rectangularBlock(1, 0.5, 0.5);
    gripper3 = mesh_rectangularBlock(1, 0.5, 0.5);  
    gripper4 = mesh_rectangularBlock(1, 0.5, 0.5);
    gripper5 = mesh_rectangularBlock(1, 0.5, 0.5);  
    gripper6 = mesh_rectangularBlock(1, 0.5, 0.5);
    gripper7 = mesh_rectangularBlock(1, 0.5, 0.5);  
    gripper8 = mesh_rectangularBlock(1, 0.5, 0.5);
    gripper1.u = [1; .25; .25];
    gripper2.u = [-1; .25; .25];
    gripper3.u = [1; .25; -.25];
    gripper4.u = [-1; .25; -.25];
    gripper5.u = [1; -.25; .25];
    gripper6.u = [-1; -.25; .25];
    gripper7.u = [1; -.25; -.25];
    gripper8.u = [-1; -.25; -.25];
    gripper1.mu = 1;
    gripper2.mu = 1;  
    gripper3.mu = 1;
    gripper4.mu = 1;  
    gripper5.mu = 1;
    gripper6.mu = 1;  
    gripper7.mu = 1;
    gripper8.mu = 1;  
    gripper1.color = [1 0 0];
    gripper2.color = [1 0 0];
    gripper3.color = [1 0 0];
    gripper4.color = [1 0 0];
    gripper5.color = [1 0 0];
    gripper6.color = [1 0 0];
    gripper7.color = [1 0 0];
    gripper8.color = [1 0 0];
    w1.u = [ 1.5; .25; .25];
    w2.u = [-1.5; .25; .25];
    w3.u = [ 1.5; .25; -.25];
    w4.u = [-1.5; .25; -.25];
    w5.u = [ 1.5; -.25; .25];
    w6.u = [-1.5; -.25; .25];
    w7.u = [ 1.5; -.25; -.25];
    w8.u = [-1.5; -.25; -.25];
    
    % Add bodies to simulator
    sim = sim_addBody(sim, [ground box gripper1 gripper2 gripper3 gripper4 gripper5 gripper6 gripper7 gripper8]);
    
    % Create joints
     sim = sim_addJoint( sim, 1, 3, w1.u, [1;0;0], 'prismatic');
     sim = sim_addJoint( sim, 1, 4, w2.u, [1;0;0], 'prismatic');
     sim = sim_addJoint( sim, 1, 5, w3.u, [1;0;0], 'prismatic');
     sim = sim_addJoint( sim, 1, 6, w4.u, [1;0;0], 'prismatic');
     sim = sim_addJoint( sim, 1, 7, w5.u, [1;0;0], 'prismatic');
     sim = sim_addJoint( sim, 1, 8, w6.u, [1;0;0], 'prismatic');
     sim = sim_addJoint( sim, 1, 9, w7.u, [1;0;0], 'prismatic');
     sim = sim_addJoint( sim, 1, 10, w8.u, [1;0;0], 'prismatic');

     % disable collisions between bodies
     gripper1.doesNotCollideWith = [gripper3.bodyID gripper5.bodyID gripper7.bodyID];
     gripper3.doesNotCollideWith = [gripper1.bodyID gripper5.bodyID gripper7.bodyID];
     gripper5.doesNotCollideWith = [gripper1.bodyID gripper3.bodyID gripper7.bodyID];
     gripper7.doesNotCollideWith = [gripper1.bodyID gripper3.bodyID gripper5.bodyID];
     gripper2.doesNotCollideWith = [gripper4.bodyID gripper6.bodyID gripper8.bodyID];
     gripper4.doesNotCollideWith = [gripper2.bodyID gripper6.bodyID gripper8.bodyID];
     gripper6.doesNotCollideWith = [gripper2.bodyID gripper4.bodyID gripper8.bodyID];
     gripper8.doesNotCollideWith = [gripper2.bodyID gripper4.bodyID gripper6.bodyID];
    
    % Run the simulator
    sim = sim_run_Drumwright ( sim );

end

