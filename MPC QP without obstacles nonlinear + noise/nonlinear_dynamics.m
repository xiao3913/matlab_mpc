function z = nonlinear_dynamics(z,u,params,j)

z(1) = z(1) + (params.v * cos(z(3))) * params.Ts;
z(2) = z(2) + (params.v * sin(z(3))) * params.Ts;
z(3) = z(3) + (params.v * tan(u(j)) / params.l) * params.Ts;

end