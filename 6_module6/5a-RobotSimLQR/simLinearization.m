
params = struct();

params.g = 9.81;
params.mr = 0.25;
params.ir = 0.0001;
params.d = 0.1;
params.r = 0.02;

g = params.g;
mr = params.mr;
ir = params.ir;
d = params.d;
r = params.r;

% 1. Learn how to use the symbolic toolbox in MATLAB
% you will need the state variables, and control input to be declared as symbolic

% 2. call your "eom" function to get \ddot{q} symbolically
syms th phi dth dphi u

% Define state vector q and its time derivatives
q = [th; phi; dth; dphi];

qdot = [dth
        dphi
        -(ir*u + mr*r^2*u + d^2*mr*u*cos(phi)^2 + d^2*mr*u*sin(phi)^2 - d^3*dphi^2*mr^2*r*sin(phi)^3 + d*g*mr^2*r^2*sin(phi) + 2*d*mr*r*u*cos(phi) - d^2*dphi^2*mr^2*r^2*cos(phi)*sin(phi) - d^3*dphi^2*mr^2*r*cos(phi)^2*sin(phi) + d^2*g*mr^2*r*cos(phi)*sin(phi) - d*dphi^2*ir*mr*r*sin(phi))/(mr*r^2*(mr*d^2*sin(phi)^2 + ir))
        (r*u + d*u*cos(phi) + d*g*mr*r*sin(phi) - d^2*dphi^2*mr*r*cos(phi)*sin(phi))/(r*(mr*d^2*sin(phi)^2 + ir))];


% 3. Linearize the system at 0 (as shown in lecture)
% You should end up with A (4x4), and b (4x1)
A_Jacob = jacobian(qdot, q);

eq_point = [0; 0; 0; 0];
A = subs(A_Jacob, q, eq_point);
A = double(A)

B_Jacob = jacobian(qdot, u);
B = subs(B_Jacob, q, eq_point);
B = double(B)

% 4. Check that (A,b) is  controllable
% Number of uncontrollable states should return 0

%B = [0; 0; 0; 1];  % Assuming control input is applied to dphi
CtrbMatrix = [B, A*B, A^2*B, A^3*B];
uncontrollable_states = length(q) - rank(CtrbMatrix);


% 5. Use LQR to get K as shown in the lecture
Q = eye(4);  % You may need to adjust this based on your system
R = 100000;        % You may need to adjust this based on your system
K = lqr(A, B, Q, R);

disp('Jacobian matrix A:');
disp(A);
disp('Controllability Matrix:');
disp(CtrbMatrix);
disp('Number of uncontrollable states:');
disp(uncontrollable_states);
disp('LQR Controller Gain K:');
disp(K);


