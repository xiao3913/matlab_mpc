function z_new = car_sim(z_curr,u_curr,params)

delta_f        = u_curr(1);

% vehicle dynamics equations

params.psis = 0;
params.Xs = z_curr(6);
params.Ys = 0;

z_new(4) = z_curr(4) + tan(z_curr(5)) * params.Ts * params.v0;
z_new(5) = z_curr(5) + (tan(delta_f)/((params.l_r + params.l_f)*cos(z_curr(5)))) * params.Ts * params.v0;
z_new(6) = z_curr(6) + (params.v0*cos(z_curr(5)))*params.Ts;

z_new(1) = z_new(6);
z_new(2) = z_new(4);
z_new(3) = params.psis + z_new(5);

end