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
  qdd = [(mr*d^3*dphi^2*cos(phi)^2*sin(phi) + mr*d^3*dphi^2*sin(phi)^3 + mr*r*d^2*dphi^2*cos(phi)*sin(phi) - g*mr*d^2*cos(phi)*sin(phi) + ir*d*dphi^2*sin(phi) - g*mr*r*d*sin(phi))/(r*(mr*d^2*sin(phi)^2 + ir))
      (- mr*cos(phi)*sin(phi)*d^2*dphi^2 + g*mr*sin(phi)*d)/(mr*d^2*sin(phi)^2 + ir)];

end