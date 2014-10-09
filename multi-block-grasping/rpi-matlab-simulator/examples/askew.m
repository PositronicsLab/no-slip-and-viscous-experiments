function omega = askew(R)
  omega(1) = 0.5*(R(3,2) - R(2,3));
  omega(2) = 0.5*(R(1,3) - R(3,1));
  omega(3) = 0.5*(R(2,1) - R(1,2));

