%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% LCPdynamics.m
%
% Constructs matrices A and b, representing the LCP of the current time step.

function [newNU, sim] = LCPdynamics( sim )

   nc = length(sim.contacts); 
   njc = sim.num_jointConstraints;
   nd = sim.num_fricdirs; 

    M = sim.dynamics.M;
   Gn = sim.dynamics.Gn;
   Gb = sim.dynamics.Gb; 
   if ~sim.FRICTION
        error('This model is only applicable for frictional contact dynamics\n');   else 
        Gf = sim.dynamics.Gf;
        U = sim.dynamics.U;
   end
   NU = sim.dynamics.NU;
   FX = sim.dynamics.FX;
   PSI = sim.dynamics.PSI;
   psi_b = sim.dynamics.joint_bn; 
    h = sim.h;

   % special case
   if (nc == 0)
     newNU = NU + h*M \ FX
     return;
   end

   % setup the objective function 
   H = [Gn'; Gf'] * (M \ [Gn Gf]);
   c = [Gn'; Gf'] * (NU + h*(M \ FX));

   % setup the normal constraint (A*x >= b)
   A = Gn' * (M \ [Gn Gf]);
   b = -Gn' * (NU + h*(M \ FX));% - PSI/h;

   % setup the l1-norm Coulomb constraint
   nk = size(Gf,2)/nc;
   for i=1:nc
     A(size(A,1)+1,:) = zeros((1+nk)*nc,1);
     A(size(A,1),i) = U(i,i);
     b(length(b)+1) = 0; 
    for j=1:nk
       A(size(A,1),nc+nk*(i-1)+j) = -1.0;
     end
   end

   % setup the QP as a LCP
   if (size(b,2) > 1)
     b = b';
   end

   MM = [H -A'; A zeros(size(A,1))];
   qq = [c; -b]; 

   % Solve with Lemke 
   if isequal(sim.H_solver,@Lemke)
     z0 = zeros(length(qq),1); 
     tstart = tic();
     [lcpSoln,err] = lemke(MM,qq,z0);  
     w = MM*lcpSoln+qq;

     % check solution
     epsilon = 1e-16;
     while (epsilon < 1e-6)
       if (min(lcpSoln) > -1e-4 && min(w) > -1e-4)
         break;
       end
       MMprime = MM + eye(size(MM))*epsilon;
       [lcpSoln,err] = lemke(MMprime, qq, z0);
       w = MM*lcpSoln+qq;
       epsilon = epsilon * 2;
     end
     telapsed = toc(tstart)
     eps
     sim.solution_error = [sim.solution_error max([0 -min([lcpSoln w])])];
   elseif isequal(sim.H_solver, @pgs)
     lcpSoln = pgs(MM, qq, zeros(length(qq), 1), 100);
     w = MM*lcpSoln+qq;
     sim.solution_error = [sim.solution_error max([0 -min([lcpSoln w])])];
   else
     opt = optimset('Algorithm', 'interior-point-convex', 'MaxIter', 1000, 'TolFun', 1e-6, 'TolCon', 1e-6);
     [lcpSoln,fval,exitflag,output] = quadprog(H, c, -A, -b, zeros(0,size(A,2)), [], zeros((nk+1)*nc,1), [],[], opt);
     size(M)
     w = H*lcpSoln+c;
     min(w)
     w'*lcpSoln
   end   

   % get the impulses out
   cn = lcpSoln(1:nc);
   cf = lcpSoln(nc+1:nc+nk*nc);

   % Determine newNU
   newNU = M \ (Gn*cn + Gf*cf + h*FX);
    

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



