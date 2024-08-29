
function u = controller(params, t, phi, phidot)
  % STUDENT FILLS THIS OUT
  % 
  % Initialize any added state like this:
  persistent integr t_prev
  
  if isempty(integr)
    % initialize
    integr = 0;
    t_prev = t;
  end
  
  dt = t - t_prev;
  t_prev = t;
  
  kp=40;
  kd=2;
  ki=1000;
  integr=integr + (phi*dt);
  
  u= -(kp*phi + kd*phidot + ki*integr);
  u= -u; %the grader has a different convention
 
  
end
