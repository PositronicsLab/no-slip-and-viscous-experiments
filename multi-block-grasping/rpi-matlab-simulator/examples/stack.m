function [sim, mmse] = stack(nboxes, iter )

    % clear mmse
    mmse = 0;

    %% A user function for controlling the wheel torque to achieve a certain speed.
    % userFunctions get called at the beginning of every timestep.
    function sim = updateMSE( sim )

      % compute error over position of each box
      for i=1:nboxes
        mmse = mmse + norm(sim.bodies(i+1).u - [0; 0; i-0.5]);
      end
    end

    %% Our main function that creates the stack 
    sim = Simulator(.01);
    sim.MAX_STEP = iter;
    sim.userFunction = @updateMSE;
    sim.H_dynamics = @LCPdynamics;
    sim.H_dynamics = @Drumwrightdynamics;
    sim.H_solver = @pgs;
    sim.H_solver = 0;@Lemke;
    %sim.drawContacts = true;
    sim.FRICTION = true; 
    sim.num_fricdirs = 4; 
    sim.drawJoints = false; 

    % ground
    ground = Body_plane([0; 0; 0], [0;0;1]);
      ground.color = [0 0 0];
      ground.dynamic = false;
      ground.visible = false;   
 
    % object 
    for i=1:nboxes
      box(i) = mesh_rectangularBlock(1, 1, 1);
      box(i).u = [0; 0; i-0.5];
      box(i).color = rand(3,1); 
      box(i).mu = 1;
      box(i).J = diag([1 1 1]);
    end

    fprintf(1,'Adding bodies to simulator\n');
    % Add bodies to simulator
    sim = sim_addBody(sim, [ground box]);
    fprintf(1,'About to run simulator\n');
    
    % Run the simulator
%    sim = sim_run_Drumwright( sim );
    sim = sim_run( sim );

    % compute mean
    mmse = mmse / (nboxes * iter);

end

