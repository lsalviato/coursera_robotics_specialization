
function Xend = simode(beta, tend)
  % this parameter is set by the calling script
  params.delta = 1;
  params.alpha = 1;
  params.beta = beta;
  params.gamma = 1;
  params.omega = 1;
  % this is the initial condition
  X0 = [1 1]';

  f_duff = @(t,x,params) [x(2); (params.gamma*cos(params.omega*t) -params.delta*x(2) -params.alpha*x(1) -params.beta*x(1)^3) ];

  [t,y] = ode45(@(t,x) f_duff(t,x,params) ,[0 tend]', X0);
  % student fills this out
  % You should write code that integrates the ode from time 0 to tend, and assigns "Xend" such that Xend = X at time tend.
  Xend = y(end,:);

  % The following line should help plot the solutions if you assign to X the second output of ode45
  % plot(X(:,1), X(:,2))
end

function Xd = dyn(params, t, X)
  x = X(1);
  xd = X(2);

  % student completes this
  % Note: you have all the parameters available here, e.g. params.alpha
  Xd = zeros(size(X));
end
