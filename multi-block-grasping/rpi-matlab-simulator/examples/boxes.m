function [sim, ke] = boxes( )

    % the kinetic energy of the box with respect to the grippers 
    ke = [];

    %% A user function for controlling the wheel torque to achieve a certain speed.
    % userFunctions get called at the beginning of every timestep.  
    function sim = applyForceFunction( sim )

        % compute predynamics
        sim = preDynamics(sim);

        % get the relative velocities of the box with respect to both grippers
        M = sim.dynamics.M;
        R = [sim.dynamics.Gn sim.dynamics.Gb];
        A = R'*M*R;
        b = R'*sim.dynamics.NU;

        % update the ke
        ke = [ke; 0.5*dot(b, A*b)]; 
    end

    %% Our main function that creates a chassis with four wheels 
    sim = Simulator(.01);
    sim.MAX_STEP = 1000;
    sim.userFunction = @applyForceFunction;                 % Tell sim to use our userFunction
    sim.H_dynamics = @LCPdynamics;
    sim.H_solver = @pgs;
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
    ii = 1;
    for i=1:5
      for j=1:5
        for k=1:5
          box(ii) = mesh_rectangularBlock(1, 1, 1);
          box(ii).u = [i-5; j-5; k-.5];
          box(ii).color = rand(3,1); 
          box(ii).mu = 1;
          ii = ii + 1;
        end
      end
    end

    % Add bodies to simulator
    sim = sim_addBody(sim, [ground box]);
    
    % Run the simulator
%    sim = sim_run_Drumwright( sim );
    sim = sim_run( sim );

end

