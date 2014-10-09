function [sim, ke] = graspPenalty( )

    % the kinetic energy of the box with respect to the grippers 
    ke = [];

    %% A user function for controlling the wheel torque to achieve a certain speed.
    % userFunctions get called at the beginning of every timestep.  
    function sim = applyForceFunction( sim )
        F = 10;
        kp = 100;
        kv = 10;
        avelD = 1;

        % convert the quaternions on bodies 2 and 3 to rotation matrices
        R2 = qt2rot(sim.bodies(2).quat);        
        R3 = qt2rot(sim.bodies(3).quat);        

        % apply force to keep blocks along x-axis
        err2 = [0 -sim.bodies(2).u(2) -sim.bodies(2).u(3) -askew(R2)]';
        derr2 = [0; -sim.bodies(2).nu(2:6)];
        err3 = [0 -sim.bodies(3).u(2) -sim.bodies(3).u(3) -askew(R3)]';
        derr3 = [0; -sim.bodies(3).nu(2:6)];

        % apply torque to spin wall
%        sim.bodies(1).Fext(6) = kv * (avelD - sim.bodies(1).nu(6));

        % apply forces pushing in
        fx = [F 0 0]; 
        sim.bodies(2).Fext(1:3) = -fx';          
        sim.bodies(3).Fext(1:3) = fx';          

        % apply forces to keep grippers constrained
        sim.bodies(2).Fext = sim.bodies(2).Fext + (kp*err2 + kv*derr2);
        sim.bodies(3).Fext = sim.bodies(3).Fext + (kp*err3 + kv*derr3);

        % get the relative velocities of the box with respect to both grippers
        rv1 = sim.bodies(1).nu - sim.bodies(2).nu;
        rv2 = sim.bodies(1).nu - sim.bodies(3).nu;

        % update the ke
        ke = [ke; 0.25*(dot(rv1,rv1) + dot(rv2,rv2))];
    end


    %% Our main function that creates a chassis with four wheels 
    sim = Simulator(.001);
    sim.MAX_STEP = 1000;
    sim.userFunction = @applyForceFunction;                 % Tell sim to use our userFunction
    sim.penalty_integrator = @ode23s;
    sim.H_dynamics = @penalty;
    sim.H_solver = @lemke;
    %sim.drawContacts = true;
    sim.FRICTION = true; 
    sim.num_fricdirs = 4; 
    sim.drawJoints = true; 

    % object 
    box = mesh_rectangularBlock(1.25, 0.75, 0.75);
        box.u = [0; 0; 0];
        box.color = [.3 .6 .5];
        box.mu = 1;

    % apply a random velocity to the box
    box.nu = randn(6,1);
        
    % setup manipulator 
    gripper1 = mesh_cube();  
    gripper2 = mesh_cube();
    gripper1.u = [1; 0; 0];
    gripper2.u = [-1; 0; 0];
    gripper1.mu = 1;
    gripper2.mu = 1;  
    gripper1.color = [1 0 0];
    gripper2.color = [1 0 0];
    
    % Add bodies to simulator
    sim = sim_addBody(sim, [box gripper1 gripper2]);
   
    % Run the simulator
    sim = sim_run_penalty( sim );

end

