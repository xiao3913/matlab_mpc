function obs_avoidance(~, msg)
global cmd_pub count s0

x = msg.Pose.Pose.Position.X;                                           % Extract most recent X - coordinate from MOCAP
y = msg.Pose.Pose.Position.Y;                                           % Extract most recent Y - coordinate from MOCAP
quat = msg.Pose.Pose.Orientation;                                       % Extracts orientation in quaternion from MOCAP
eul = quat2eul([quat.W quat.X quat.Y quat.Z]);                          % Converts from quaternions to euler angles [rad]
psi = eul(1);                                                           % Sets heading angle [rad]
vx  = msg.Twist.Twist.Linear.X;                                         % Extracts x component of velocity from MOCAP [m/s]
vy  = msg.Twist.Twist.Linear.Y;                                         % Extracts y component of velocity from MOCAP [m/s]
params.v0 = sqrt(vx^2 + vy^2);                                          % Sets vehicles velocity [m/s]

addpath(genpath('/home/mma/Downloads/Obstacle_avoidance/YALMIP-master/solvers'));      % Sets directory to YALMIP folder

%% Parameters definition
% model parameters
params.delta_max                            = pi/4;                     % Front wheel steering angle limit [radians]
params.l_f                                  = 0.16;                     % Distance between center of gravity and front axle [m]
params.l_r                                  = 0.16;                     % Distance between center of gravity and rear axle [m]
params.l                                    = params.l_f + params.l_r;  % Wheelbase [m]
params.vehicle_width                        = 0.24;                     % Width of vehicle [m]
params.Ts                                   = 0.2;                      % Sampling time (both of MPC and simulated vehicle) [s]
params.rou                                  = inf;                      % Radius of curvature [m]               

% control parameter
params.horizon                              = 4;                                                 % Prediction horizon [-]
params.yalmip_opts                          = sdpsettings('solver', 'OSQP', 'verbose', 0);        % Solver settings

% simulation parameters
params.x0                                   = x;                        % Initial x coordinate
params.y0                                   = y;                        % Initial y coordinate
%params.v0                                   = 0.3;                      % speed [m/s]
params.psi0                                 = psi;                      % Initial heading angle [radians]
[params.ey0,params.epsi0]                   = global2curve(params);     % Converts global x, y to curvelinear coorinates

%disp('y-error : ')
%disp(params.ey0)
%disp('heading error: ')
%disp(params.epsi0)
%% Simulation environment
% initialization
z                                           = [params.x0, params.y0, params.psi0, params.ey0, params.epsi0, s0];            % Definition of the intial state                                                  
Q1                                          = 25;                                                                           % Weight for lateral deviation [-]
Q2                                          = 0;                                                                            % Weight for deviation in heading [-]

    tic()                                                               % Start counter
    yalmip('clear')
    
    % Setup optimization variables
    u_mpc = sdpvar(params.horizon,1);                                   % Sets optimization variable steering angle
    z_mpc = sdpvar(params.horizon,2);                                   % Sets optimizations variables states
    constraints = [];                                                   % Initial value for constraints
    objective   = 0;                                                    % Initial value for objective function
    
    % Define the constraints over the prediction window
    t = toc
    for k = 1:params.horizon
        
        constraints = cons(constraints,params,k,z_mpc,z,u_mpc);                                   % Vehicle dynamics
        constraints = [constraints, -params.delta_max <= u_mpc(k) <= params.delta_max];           % Constraint on steering angle      
        objective = objective + Q1*z_mpc(k,1)^2 + Q2*z_mpc(k,2)^2;                                % Objective function
    end
    
    
    
    optimize(constraints,objective,params.yalmip_opts);                                           % Calls optimizer
    
    u_pred  = double(u_mpc); % Predicted control inputs
    
    u = u_pred(1) * 180/pi;
    
    %disp('steering angle:')
    %disp(u)
   % Control input [degrees]
   
    if u >= 0
         
         angle = int16(1495 - 5.016 * u - 0.2384 * u^2);
         
     elseif u < 0
         
         angle = int16(1495 - 5.016 * u + 0.2384 * u^2);
     end
     
     if angle >= 2100
         
         angle = 2100;
         
     elseif angle <= 900
         
         angle = 900;
     end
    
     cmdmsg = rosmessage(cmd_pub);       % Creates message for commands (velocity and angle)
     cmdmsg.Linear.X = 1600; %params.v0;        % Set velocity of vehicle
     cmdmsg.Angular.Z = angle;           % Set steering angle
     send(cmd_pub, cmdmsg);              % Send commands to vehicle
     
     s0 = s0 + params.Ts * params.v0 * cos(params.epsi0);           % Predicts distance covered [m]
     count = count + 1;
                                                            % Calculates time elapsed since tic()
end