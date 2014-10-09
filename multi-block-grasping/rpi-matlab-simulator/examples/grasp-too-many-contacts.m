function sim = grasp( )

    %% A user function for controlling the wheel torque to achieve a certain speed.
    % userFunctions get called at the beginning of every timestep.  
    function sim = applyForceFunction( sim )
        F = 10;

        % apply forces 
        sim.bodies(3).Fext(1) = -F;          
        sim.bodies(4).Fext(1) = F;          
    end


    %% Our main function that creates a chassis with four wheels 
    sim = Simulator(.01);
    sim.userFunction = @applyForceFunction;                 % Tell sim to use our userFunction
    sim.H_dynamics = @mLCPdynamics;
    sim.H_solver = @lemke;
    %sim.drawContacts = true;
    sim.FRICTION = true; 
    sim.num_fricdirs = 4; 
    sim.drawJoints = true; 
    
    % Ground
    wall = Body_plane([0; 0; -1],[0.0; 0.0; 1]); 
        wall.color = [0 0 0];
        wall.dynamic = false;
    
    % object 
    box = mesh_cube();
        box.u = [0; 0; 0];
        box.color = [.3 .6 .5];
        box.mu = 1;
        
    % setup manipulator 
    gripper1 = mesh_cube();  
    gripper2 = mesh_cube();
    gripper1.u = [1; 0; 0];
    gripper2.u = [-1; 0; 0];
    gripper1.mu = 1;
    gripper2.mu = 1;  
    gripper1.color = [1 0 0];
    gripper2.color = [1 0 0];
    w1.u = [ 1.5;  0; 0];  
    w2.u = [-1.5;  0; 0]; 
    
    % Add bodies to simulator
    sim = sim_addBody(sim, [wall box gripper1 gripper2]);
    
    % Create joints
     sim = sim_addJoint( sim, 1, 3, w1.u, [1;0;0], 'prismatic');
     sim = sim_addJoint( sim, 1, 4, w2.u, [1;0;0], 'prismatic');
    
    % Run the simulator
    sim = sim_run( sim );

end

