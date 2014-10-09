%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% LCPdynamics.m
%
% Constructs matrices A and b, representing the LCP of the current time step.

function newNU = LCPdynamics( sim )

   nb = sim.num_activeBodies; 
   nc = length(sim.contacts); 
   njc = sim.num_jointConstraints;
   nd = sim.num_fricdirs; 

    M = sim.dynamics.M;
   Gn = sim.dynamics.Gn;
   Gb = sim.dynamics.Gb; 
   if sim.FRICTION
        Gf = sim.dynamics.Gf;
        U = sim.dynamics.U;
        E = sim.dynamics.E;
   end
   NU = sim.dynamics.NU;
   FX = sim.dynamics.FX;
  PSI = sim.dynamics.PSI;
   psi_b = sim.dynamics.joint_bn; 
    h = sim.h;
    
   QA = [[M;Gb'] [-Gb;zeros(njc,njc)]];
   QB = [[-Gn;zeros(njc,nc)] [-Gf;zeros(njc,nc*nd)] [zeros(6*nb+njc, nc)]];
   QE = [-M*NU-FX*h; psi_b];
   QF = [PSI/h;zeros(nc*nd, 1);zeros(nc, 1)];
   QM = [[Gn'; Gf'; zeros(nc,6*nb)] [zeros(nc*(nd+2),njc)]];
   QN = [[zeros(nc*(nd+1), nc); U] [zeros(nc*(nd+1), nc*nd); -E'] [zeros(nc, nc); E; zeros(nc, nc)]];

   QAinv = inv(QA);
   MM = QN - QM*QAinv*QB;
   qq = QF - QM*QAinv*QE;
    
   % Solve with Lemke 
   z0 = zeros(length(qq),1); 
   [lcpSoln,err] = lemke(MM,qq,z0);  
   
   % Determine newNU
   QD = lcpSoln;
   QC = QAinv * (-QB*QD - QE);
   newNU = QC(1:6*nb);
   
    

  %% Construct A and b as LCP
%   if sim.FRICTION
%       MinvGn = M\Gn;
%       MinvGf = M\Gf;
% 
%       A = [ Gn'*MinvGn   Gn'*MinvGf  zeros(nc)
%             Gf'*MinvGn   Gf'*MinvGf  E
%             U            -E'         zeros(nc)];
% 
%       b = [ Gn'*(NU + M\FX*h) + PSI/h;    % FX*h could be replaced if we stored Pext instead of Fext
%             Gf'*(NU + M\FX*h);
%             zeros(nc,1) ];
%       
%       % Solve with LEMKE
%       z = lemke( A, b, zeros(length(b),1) ); 
% 
%       % Calculate updated velocities. 
%       % The impulse in the normal direction per active body.
%       Pn = z(1:nc);
%       % The impulse in the friction directions per active body.
%       Pf = z(nc+1:nc + sim.num_fricdirs*nc);
%       % Calculate new velocites
%       newNU = NU + MinvGn*Pn + MinvGf*Pf + M\FX*h;
%   
%   else  % Same but without Gf
%       MinvGn = M\Gn;
% 
%       A = Gn'*MinvGn;
% 
%       b = [ Gn'*(NU + M\FX*h) + PSI/h;    % FX*h could be replaced if we stored Pext instead of Fext
%             zeros(nc,1) ];
%       
%       % Solve with LEMKE
%       z = lemke( A, b, zeros(length(b),1) ); 
% 
%       % Calculate updated velocities. 
%       % The impulse in the normal direction per active body.
%       Pn = z(1:nc);
%       % The impulse in the friction directions per active body.
%       Pf = z(nc+1:nc + sim.num_fricdirs*nc);
%       % Calculate new velocites
%       newNU = NU + MinvGn*Pn + M\FX*h;
%       
%   end
 

  
end



