
function xhat = EKFstudent(t, z)
  % In this exercise, you will batch-process this data: you are provided a vector of timestamps (of length T), and a 3xT matrix of observations, z.
  xhat = zeros(2,length(t));

  z(3,:) = deg2rad(z(3,:)); 

  % Student completes this

  % noise covariance matrices
  Q = 10*diag([1e-3 1]); %state update noise
  R = diag([1 1 1]); %measurement noise

  %initialize state and state covariance 
  P = 10*eye(2);
  x = [0 0]';
  
  xhat(:,1) = x;
  dt = diff(t);
  for i=2:length(t)
      % define system matrices
      A = [1 t(i)-t(i-1);
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

      x = x_k_kminus + K*(z(:,i) - h);
      P = (eye(2) - K*H)*P_k_kminus;

      %save estimate
      xhat(:,i) = x;
  end

xhat = rad2deg(xhat);
end


%% code to calcuate jacobian
% syms phi phi_dot
% 
% h1 = sin(phi);
% h2 = cos(phi);
% h3 = phi_dot;
% 
% h = [h1; h2; h3];
% 
% J = jacobian(h, [phi, phi_dot]);
% disp(J);
