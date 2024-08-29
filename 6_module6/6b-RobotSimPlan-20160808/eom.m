function qdd = eom(params, th, phi, dth, dphi, u)
  % This is the starter file for the week5 assignment

  % Provided params are
  % params.g: gravitational constant
  % params.mr: mass of the "rod"
  % params.ir: rotational inertia of the rod
  % params.d: distance of rod CoM from the wheel axis
  % params.r: wheel radius

  % Provided states are:
  % th: wheel angle (relative to body)
  % phi: body pitch
  % dth, dphi: time-derivatives of above
  % u: torque applied at the wheel
  % THE STUDENT WILL FILL THIS OUT

  g = params.g;
  mr = params.mr;
  ir = params.ir;
  d = params.d;
  r = params.r;
  
  % dynamics calculated using MyScript_lagrangian_calc.m
  qdd = [  -(ir*u + mr*r^2*u + d^2*mr*u*cos(phi)^2 + d^2*mr*u*sin(phi)^2 - d^3*dphi^2*mr^2*r*sin(phi)^3 + d*g*mr^2*r^2*sin(phi) + 2*d*mr*r*u*cos(phi) - d^2*dphi^2*mr^2*r^2*cos(phi)*sin(phi) - d^3*dphi^2*mr^2*r*cos(phi)^2*sin(phi) + d^2*g*mr^2*r*cos(phi)*sin(phi) - d*dphi^2*ir*mr*r*sin(phi))/(mr*r^2*(mr*d^2*sin(phi)^2 + ir))
           (r*u + d*u*cos(phi) + d*g*mr*r*sin(phi) - d^2*dphi^2*mr*r*cos(phi)*sin(phi))/(r*(mr*d^2*sin(phi)^2 + ir))];


end