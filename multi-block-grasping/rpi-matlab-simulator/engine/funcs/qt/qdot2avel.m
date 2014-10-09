% Converts a time derivative of a quaternion to an angular velocity vector 

function w = avel2qdot(q, qdot)

  % determine G
  G = [-q(2) q(1) -q(4) q(3); -q(3) q(4) q(1) -q(2); -q(4) -q(3) -q(2) q(1)];

  % compute angular velocity 
  w = 0.5*G*qdot;

