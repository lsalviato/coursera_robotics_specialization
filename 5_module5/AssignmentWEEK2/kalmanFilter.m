function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )

    if previous_t < 0
        state = [x; y; 0; 0];
        param.P = 2 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    dt = t - previous_t;
    A = [1, 0, dt, 0;
         0, 1, 0, dt;
         0, 0, 1, 0;
         0, 0, 0, 1];
    C = [1, 0, 0, 0;
         0, 1, 0, 0];

   
    Omega_m = [dt*dt/4, 0, dt/2, 0;
               0, dt*dt/4, 0, dt/2;
               dt/2, 0, 1, 0;
               0, dt/2, 0, 1];

    Omega_o = [0.01, 0;
               0, 0.01];

    P = A * param.P * A' + Omega_m;
    R = Omega_o;

    zt = [x; y];

    
    K = P * C' * inv(R + C * P * C');
    
    
    state = A * state + K * (zt - C * A * state);
    param.P = P - K * C * P;

    predictx = state(1) + state(3) * 0.330;
    predicty = state(2) + state(4) * 0.330;
end