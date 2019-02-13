function z = nonlinear_dynamics(z,u,params,j)

z(1) = z(1) + ((params.rou - z(1)) / params.rou * tan(z(2))) * params.deltas;
z(2) = z(2) + (tan(u(j)) * (params.rou - z(1)) / (params.rou * params.l * cos(z(2))) - 1/params.rou) * params.deltas;
z(3) = z(3) + (params.rou * params.v * cos(z(2)) / (params.rou - z(1))) * params.Ts;

z = z';
end