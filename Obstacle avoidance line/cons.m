function [constraints] = cons(constraints,params,k,z_mpc,z,u_mpc)

A = [1                                                   params.v0*params.Ts;
     -tan(0)/(params.rou*params.l)*params.v0*params.Ts             1];

B = [0   params.v0*params.Ts/(params.l*cos(0)^2)]';

C = [0   (tan(0)/params.l-1/params.rou-0/(params.l*cos(0)^2))*params.v0*params.Ts]';


if k == 1
    
    constraints = [constraints, z_mpc(k,1:end)' ==  A * z(1,4:5)' + B * u_mpc(k) + C];
    
else
    constraints = [constraints, z_mpc(k,1:end)' == A * z_mpc(k-1,1:end)' + B * u_mpc(k) + C];
    
end