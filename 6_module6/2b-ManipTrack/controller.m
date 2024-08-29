
function u = controller(params, t, X)
  u=[0; 0];
  % 1. write out the forward kinematics, such that p = FK(theta1, theta2)
  % 2. Let e = p - params.traj(t) be the task-space error
  % 3. Calculate the manipulator Jacobian J = d p / d theta
  % 4. Use a "natural motion" PD controller, u = - kp * J^T * e - kd * [dth1; dth2]

  th1 = X(1);
  th2 = X(2);
  L1 = params.l;
  L2 = params.l;
  
  %end effector position
  p = [L1*cos(th1) + L2*cos(th1+th2);
     L1*sin(th1) + L2*sin(th1+th2)];

 
  %Jacobian
  J = [- L2*sin(th1 + th2) - L1*sin(th1), -L2*sin(th1 + th2)
         L2*cos(th1 + th2) + L1*cos(th1),  L2*cos(th1 + th2)];

  %end effector error
  e = p - params.traj(t);

  kp = 10000;
  kd = 30]

  %stabilizing feedback control law
  u = - kp * J' * e - kd * X(3:4);
  
end
