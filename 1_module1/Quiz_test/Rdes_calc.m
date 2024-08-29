%% calculate desired rotation matrix from the thrust vector
t = [sind(30)*cosd(45); sind(30)*sind(45); cosd(30)] %thrust vector(versor)
psi = 45; %yaw angle
deg = pi/180;

%from the R matrix in the expanded form, since R*b3 = r / ||r||
%in this special case r / ||r|| = r

sin_theta = (t(1) + t(2)*tand(psi))/...
    (cosd(psi) + sind(psi)^2/cosd(psi))
theta = asind(sin_theta)
phi = acosd( t(3)/cosd(theta))

Rz = [cosd(psi)  -sind(psi)  0 
      sind(psi)   cosd(psi)  0 
      0             0            1];

Rx = [1             0            0
      0             cosd(phi) -sind(phi)
      0             sind(phi)  cosd(phi)];

Ry = [cosd(theta) 0            sind(theta)
      0             1            0            
     -sind(theta) 0            cosd(theta)];

[psi,phi,theta]
R = Rz * Rx * Ry

%% caclulate delta_R matrix from current and desired
R_des = [0 0 1; 1 0 0; 0 1 0]
R_curr = [0.7244  0.1294  0.6771;
          0.6424 -0.4830 -0.5959;
          0.2400  0.8660 -0.4330]

delta_R = R_curr'*R_des