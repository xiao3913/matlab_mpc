clear; close; clc;
addpath(genpath('/home/mma/Downloads/Obstacle_avoidance/YALMIP-master'));
%% Parameters definition
% model parameters
params.delta_max                            = pi/4;                     % Front wheel steering angle limit
params.l_f                                  = 0.16;                     % Distance between center of gravity and front axle [m]
params.l_r                                  = 0.16;                     % Distance between center of gravity and rear axle [m]
params.l                                    = params.l_f + params.l_r;  % Wheelbase [m]
params.vehicle_width                        = 0.24;                     % Width of vehicle [m]
params.Ts                                   = 0.2;                     % Sampling time (both of MPC and simulated vehicle) [s]
params.nstates                              = 6;                        % Number of states [-]
params.ninputs                              = 1;                        % Number of inputs [-]
params.rou                                  = inf;                      % Radius [m]               

% environment parameters
params.activate_obstacles                   = 0;                                                  % 0 if there are no obstacles, 1 otherwise [-]
params.obstacle_centers                     = [0.8 0.05; 1.2 -0.05; 1.7 0.04; 2.0 0.04];            % x and y coordinates of the obstacles                         
params.rot                                  = [0 0 0 0];                                          % Rotation of obstacles [degree]

params.obstacle_size                        = [0.1 0.2];                                          % Size of the obstacles (l,w) [m]
bound                                       = [-0.064 0.065 -0.083 -0.069];                       % Bounds on e_y to avoid obstacles [m]
params.lane_length                          = 2.5;                                                % Length of lane [m]

% control parameter
params.horizon                              = 5;                                                 % Prediction horizon [-]
params.yalmip_opts                          = sdpsettings('solver', 'OSQP', 'verbose', 0);

% simulation parameters
params.x0                                   = 0;                        % Initial x coordinate
params.y0                                   = 0;                        % Initial y coordinate
params.v0                                   = 0.3;                      % Initial speed [m/s]
params.psi0                                 = 0;                        % Initial heading angle [radians]
params.ey0                                  = 0;                        % Initial lateral deviation from centerline [m]
params.epsi0                                = 0;                        % Initial deviation from heading angle of centerline [radians]
params.s0                                   = 0;                        % Initial distance covered along centerline [m]
params.N_max                                = 200;                      % Maximum number of simulation steps [-]

% plotting parameters
params.window_size          = 10;                       % Plot window size for the during simulation, centered at the current position (x,y) of the vehicle
params.plot_full            = 1;                        % 0 for plotting only the window size, 1 for plotting from 0 to track_end

%% Simulation environment
% initialization
z                           = zeros(params.N_max,params.nstates);                                                  % z(k,j) denotes state j at step k-1
u                           = zeros(params.N_max,params.ninputs);                                                  % z(k,j) denotes input j at step k-1
z(1,:)                      = [params.x0, params.y0, params.psi0, params.ey0, params.epsi0, params.s0];            % Definition of the intial state
t                           = zeros(params.N_max,1);                                                               % Time to run each itteration [s] 
solve_times                 = zeros(params.N_max);                                                                 % Optimization solver time each sample [s]
z_pred                      = zeros(params.horizon,params.nstates);                                                % Definition of matrix containing predicted states
i = 1;
Q1                          = 25;                   % Weight for lateral deviation [-]
Q2                          = 0;                    % Weight for deviation in heading [-]
Q3                          = 1000;                 % Weight for slack variable [-]
while z(i,6) < params.lane_length && i<params.N_max
    tic()
    yalmip('clear')
    
    % Setup optimization variables
    u_mpc = sdpvar(params.horizon,1);
    z_mpc = sdpvar(params.horizon,3);
    %eps = sdpvar(1);                    % Definition of slack variable [-]
    constraints = [];
    %constraints = [constraints, eps >= 0];
    objective   = 0;
    
    % Define the constraints over the prediction window
    
    for k = 1:params.horizon
        
        constraints = cons(constraints,params,k,z_mpc,z,u_mpc,i);                                   % Vehicle dynamics
        constraints = [constraints, -params.delta_max <= u_mpc(k) <= params.delta_max];             % Constraint on steering angle
        
        if i > 1 && params.activate_obstacles == 1                                                  % Algorithm for obstacle avoidance
            constraints = avoidance_algorithm(constraints,eps,z_pred,z_mpc,z,bound,params,i,k);
        end
        objective = objective + Q1*z_mpc(k,1)^2 + Q2*z_mpc(k,2)^2 + Q3*eps^2;                       % Objective function
    end
    
    diagnostic     = optimize(constraints,objective,params.yalmip_opts);
    
    solve_times(i) = diagnostic.solvertime;
    
    u_pred  = double(u_mpc);                                                                        % Predicted control inputs 
    z_pred(1:params.horizon, [4 5 6]) = double(z_mpc);                                              % Predicted states ey, epsi and s
    u(i) = u_pred(1);                                                                               % Control input [radians]
    
    [z_pred(1:params.horizon,1),z_pred(1:params.horizon,2),z_pred(1:params.horizon,3)] = curve2global(params,z_pred);   % Predicted states x, y and psi
    
    z(i+1,:) = car_sim(z(i,:), u(i,:), params);                                                     % Vehicle simulation
    plot_environment(z(1:i,:), params, z_pred, u_pred);                                             % plot environment
    t(i) = toc();
    i = i+1;
end

solve_times = solve_times(solve_times>0);

racetime                    = i*params.Ts;                          % race time
fprintf('Congratulations, you completed the race in %f seconds!\n',racetime);
params.plot_full            = 1;                                    % plot the full trajectory
plot_environment(z(1:i,:),params);
