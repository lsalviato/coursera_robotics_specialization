%% my code
clc
syms th(t) phi(t)
syms g mr ir d r u real
assume(mr>0)
assume(r>0)
assume(d>0)
assume(g>0)
assume(ir>0)
assumeAlso(sin(phi(t)), 'real');
assumeAlso(cos(phi(t)), 'real');

% inverted pendulum state
q = [th phi].';

% center of mass
p = r*[th+phi 0].' + d*[sin(phi) cos(phi)].';

% kinetic energy
T = 1/2 * mr * diff(p).' * diff(p) + 1/2 * ir * diff(phi,t)^2;

% potential energy
V = mr*g*d*cos(phi);

% Lagrangian
L = T - V;

syms ddth dth ddphi dphi p t
% motion eq
%eq1 = diff(L,th) - diff(L, diff(th,t), t) == 0;
eq1 = diff(L,th) - diff(L, diff(th,t), t) == u
eq1 = expand(eq1);
eq1 = simplify(eq1);

eq1 = subs(eq1,diff(th,t,t),'ddth');
eq1 = subs(eq1,diff(th,t),'dth');
eq1 = subs(eq1,diff(phi,t,t),'ddphi');
eq1 = subs(eq1,diff(phi,t),'dphi');

eq2 = diff(L,phi) - diff(L, diff(phi,t), t) == 0;
eq2 = expand(eq2);
eq2 = subs(eq2,diff(phi,t,t),'ddphi');
eq2 = subs(eq2,diff(phi,t),'dphi');
eq2 = subs(eq2,diff(th,t,t),'ddth');
eq2 = subs(eq2,diff(th,t),'dth');

disp('pretty equation 1')
pretty(eq1)
disp('pretty equation 2')
pretty(eq2)

sol = solve([eq1, eq2], [ddth , ddphi]);
sol = subs(sol,phi,'phi')
ddth_sol = sol.ddth
ddphi_sol = sol.ddphi


