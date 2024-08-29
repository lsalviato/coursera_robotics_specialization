
function u = controllerNoisyEnc(params, t, obs, th, dth)
  % This is the starter file for the week5 assignment
  % Now you only receive noisy measurements for theta, and must use your EKF from week 3 to filter the data and get an estimate of the state
  % obs = [ay; az; gx] (same as last week)
  % New for 6b: you also have access to params.traj(t)

  % Template code (same as last week)
  xhat = EKFupdate(params, t, obs);
  phi = xhat(1);
  phidot = xhat(2);

  % The rest of this function should ideally be identical to your solution in week 4
  %% Student completes this
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
  
  u= -(kp*(phi) + kd*phidot + ki*integr);
  u= -u; %the grader has a different convention
end

function xhatOut = EKFupdate(params, t, z)
  % z = [ay; az; gx] with a* in units of g's, and gx in units of rad/s
  % You can borrow most of your week 3 solution, but this must only implement a single predict-update step of the EKF
  % Recall (from assignment 5b) that you can use persistent variables to create/update any additional state that you need.

  % Student completes this
  %state: q = [phi dphi]';

  persistent x P t_prev

  if isempty(x)
  %initialize state and state covariance 
  x = [0 0]';
  P = 10*eye(2);
  t_prev = t;
  end

  % noise covariance matrices
  Q = 10*diag([1e-3 1]); %state update noise
  R = diag([1 1 1]); %measurement noise

  % define system matrices
  A = [1 t-t_prev;
       0    1   ];
  H = [ cos(x(1))   0;
       -sin(x(1))   0;
           0        1];

  %predict
  x_k_kminus = A * x;
  P_k_kminus = A * P * A' + Q;

  %kalman gain
  K = P_k_kminus * H' / (H*P_k_kminus*H' + R);

  %measurement update
  h = [sin(x_k_kminus(1)) ;
       cos(x_k_kminus(1)) ;
       x_k_kminus(2)      ];

  x = x_k_kminus + K*(z - h);
  P = (eye(2) - K*H)*P_k_kminus;
  
  xhatOut = x;
end