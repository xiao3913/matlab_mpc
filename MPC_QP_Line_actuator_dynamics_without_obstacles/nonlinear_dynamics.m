function z = nonlinear_dynamics(z,params)
%u_diff_max = pi/1;%rate limit
%u_act = max(min(u(j),z(4)+u_diff_max),z(4)-u_diff_max);
z(1) = z(1) + tan(z(2)) * params.deltas;
z(2) = z(2) + tan(z(4)) / (params.l * cos(z(2))) * params.deltas;
z(3) = z(3) + params.v * cos(z(2)) * params.Ts;
z = [z(1) z(2) z(3)];
%z(4) = u_act;%ett sampel fördröjning
end