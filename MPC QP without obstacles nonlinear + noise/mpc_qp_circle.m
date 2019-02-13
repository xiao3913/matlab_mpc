% Model predictive control (MPC) as quadratic programming
clear, clc, close

% Model parameters
params.delta_max                    = pi/6;                           % Maximum steering angle [-]
params.l_f                          = 0.16;                           % Distance between C.O.G and front axle [m]
params.l_r                          = 0.16;                           % Distance between C.O.G and rear axle [m]
params.l                            = params.l_f+params.l_r;          % Wheelbase [m]
params.vehicle_width                = 0.24;                           % Width of vehicle [m]
params.Ts                           = 0.05;                           % Sampling time [s]
params.nstates                      = 3;                              % Number of states [-]
params.ninputs                      = 1;                              % Number of inputs [-]
params.v                            = 0.5;                            % Velocity [m/s]
params.rou                          = 1;                              % Radius [m]
params.r                            = 1;                              % Radius [m]
params.deltas                       = params.Ts*params.v;

% environment parameters
params.activate_obstacles           = 0;                                                          % 0 if there are no obstacles, 1 otherwise [-]
params.obstacle_centers             = [];                                                         % x and y coordinates of the obstacles
params.rot                          = [];                                                         % Rotation of obstacles [degree]

params.obstacle_size                = [];                                                         % Size of the obstacles (l,w) [m]
params.window_size                  = 10;                             % Plot window size for the during simulation, centered at the current position (x,y) of the vehicle
params.plot_full                    = 1;                              % 0 for plotting only the window size, 1 for plotting from 0 to end of track

% Control parameters
params.horizon                      = 20;                             % Prediction horizon [-]
q1                                  = 0.8;                              % Weight on lateral deviation [-]
q2                                  = 0.05;                              % Weight on deviation of heading angle [-]
q3                                  = 0;                              % Weight on distance covered [-]
q4                                  = 0.001;                              % Weight on input [-]
Q                                   = [q1 q2 q3];

% Simulation parameters
params.x0                           = params.r+0.5;                              % Initial x coordinate
params.y0                           = 0;                              % Initial y coordinate
params.psi                          = pi/2;                              % Initial heading angle [-]
params.ey0                          = 0;                              % Initial lateral deviation from centerline [m]
params.epsi                         = 0;                              % Initial deviation from heading angle of centerline [-]
params.s0                           = 0;                              % Initial distance covered along centerline [m]
params.N_max                        = 800;                            % Maximum number of simulation steps []
params.z0                           = [-0.5; 0; 0];            % Initial state []

% Define matrices for QP
% Construct matrix H
H                                                                                       = zeros(4*params.horizon);
H(1:(params.horizon*length(Q)),1:(params.horizon*length(Q)))                            = kron(eye(params.horizon),diag(Q));
H(params.horizon*params.nstates+1:size(H,1),params.horizon*params.nstates+1:size(H,1))  = q4*eye(params.horizon);
H                                                                                       = 2*H;


% Define constraint on input variable, steering angle
A = zeros(2*params.horizon,4*params.horizon);
A(1:params.horizon,end-params.horizon+1:end) = eye(params.horizon);                       % Upper bound
A(params.horizon+1:2*params.horizon,end-params.horizon+1:end) = -eye(params.horizon);     % Lower bound
b = params.delta_max*ones(2*params.horizon,1);                                            % Constraints on the upper and lower bounds

% MPC algorithm
zt                          = params.z0;                                     % Initial state
f                           = [];
lb                          = [];                                            % Constraints, lower bounds
ub                          = [];                                            % Constraints, upper bounds
x0                          = [];                                            % Initial state
options                     = optimset('largescale','off','display','off');
t                           = zeros(params.N_max,1);                         % Time to solve [s]
z                           = zeros(params.N_max,params.nstates+3);          % Full state vector
z_pred                      = zeros(params.horizon,params.nstates+3);        % Predicted states
u                           = zeros(params.N_max,params.ninputs);            % Steering angle [radians]
u_pred                      = zeros(params.horizon,params.ninputs);          % Predicted steering angle [radians]
z(1,:)                      = [params.x0 params.y0 params.psi zt(1) zt(2) zt(3)];         % Initial full state vector
j                           = 1;

while z(j,6) <= 2*pi*params.r
    tic
%     params.A                            = [-tan(z(j,5))/params.rou*params.deltas+1 (params.rou-z(j,4))/(params.rou*cos(z(j,5))^2)*params.deltas 0; -tan(u(j))/(params.rou*params.l*cos(z(j,5)))*params.deltas 2*sin(z(j,5))*tan(u(j))*(params.rou-z(j,4))/(cos(2*z(j,5))+1)*params.deltas+1 0; params.rou*params.v*cos(z(j,5))/(params.rou-z(j,4))^2*params.Ts -params.rou*params.v*sin(z(j,5))/(params.rou-z(j,4))*params.Ts 1];   % State prediction matrix
%     params.B                            = [0; (params.rou-z(j,4))/(cos(u(j))^2*params.rou*params.l*cos(z(j,5)))*params.deltas; 0];                             % Output prediction matrix
%     params.C                            = [((params.rou-z(j,4))/params.rou*tan(z(j,5))+tan(z(j,5))/params.rou*z(j,4)-(params.rou-z(j,4))/(params.rou*cos(z(j,5))^2)*z(j,5))*params.deltas; (tan(u(j))*(params.rou-z(j,4))/(params.rou*params.l*cos(z(j,5)))-1/params.rou+tan(u(j))*z(j,4)/(params.rou*params.l*cos(z(j,5)))-2*sin(z(j,5))*tan(u(j))*(params.rou-z(j,4))/(cos(2*z(j,5))+1)-(params.rou-z(j,4))/(cos(u(j))^2*params.rou*params.l*cos(z(j,5)))*u(j))*params.deltas; (params.rou*params.v*cos(z(j,5))/(params.rou-z(j,4))-params.rou*params.v*cos(z(j,5))/(params.rou-z(j,4))^2*z(j,4)+params.rou*params.v*sin(z(j,5))/(params.rou-z(j,4))*z(j,5))*params.Ts];              % Constant vector for system dynamics
%     
    
    params.A                            = [1 params.deltas 0; 0 1 0; params.deltas/params.rou 0 1];   % State prediction matrix
    params.B                            = [0; params.deltas/params.l; 0];                             % Output prediction matrix
    params.C                            = [0; -params.deltas/params.rou; params.deltas];              % Constant vector for system dynamics
    
    % Construct matrix AA that defines beq in the MPC - loop
    AA = zeros(3*params.horizon,params.nstates);
    
    % Construct constant vector C
    C = zeros(params.nstates*params.horizon,1);
    
    % Construct matrix Aeq that defines the feasibility of states
    Aeq = zeros(3*params.horizon,4*params.horizon);
    Aeq(1:(params.nstates*params.horizon),1:(params.nstates*params.horizon)) = eye(params.nstates*params.horizon);
    con = zeros(params.nstates*params.horizon+3,1);
    con(1:length(params.C)) = params.C;
    
    for i = 1:params.horizon
        % Construct matrix AA that defines beq in the MPC - loop
        AA(params.nstates*(i-1)+1:params.nstates*(i-1)+3,:) = params.A^i;
        
        % Construct the constant vector C
        C(params.nstates*(i-1)+1:params.nstates*(i-1)+3,:) = con(params.nstates*(i-1)+1:params.nstates*(i-1)+3);
        con(params.nstates*(i-1)+4:params.nstates*(i-1)+6) = con(params.nstates*(i-1)+1:params.nstates*(i-1)+3)+params.A^(i)*params.C;
        
        % Construct matrix Aeq that defines the feasibility of states
        Aeq(params.nstates*(i-1)+1:3*params.horizon,end-params.horizon+1:end-i+1) = Aeq(params.nstates*(i-1)+1:3*params.horizon,end-params.horizon+1:end-i+1)+kron(eye(params.horizon-i+1),-params.A^(i-1)*params.B);
    end
    
    beq             = AA*z(j,4:6)'+C;                                               % The matrix AA defines how the last measured state determines the r.h.s in the equality constraint
    x               = quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);            % Solution obtained from QUADPROG
    ut              = x(params.nstates*params.horizon+1);                    % First calculated steering angle [radians]
    zt              = params.A * zt + params.B * ut + params.C;              % Calculation of next state
    u(j)            = x(params.nstates*params.horizon+1);                    % Steering angle [radians]
    u_pred          = x(params.nstates*params.horizon+1:end);                % Predicted steering angle [radians];                                                   % State
    z_pred(:,4)     = x(1:3:params.nstates*params.horizon-2);                % Predicted lateral deviation [m]
    z_pred(:,5)     = x(2:3:params.nstates*params.horizon-1);                % Predicted deviation of heading angle w.r.t centerline [radians]
    z_pred(:,6)     = x(3:3:params.nstates*params.horizon);                  % Predicted distance covered [m]
    [z_pred(1:params.horizon,1),z_pred(1:params.horizon,2),z_pred(1:params.horizon,3)] = curve2global(params,z_pred);       % Calculates global x, y and psi from curvelinear coordinates
    z(j+1,4:6)      = nonlinear_dynamics(z(j,4:6),u,params,j);               % Nonlinear dynamics x, y and psi
    plot_environment(z(1:j,:),params,z_pred,u_pred);
    t(j)            = toc;
    j               = j+1;
end
plot_environment(z(1:j,:),params);
figure()
subplot(4,1,1)
plot(z(1:j,6),u(1:j)*180/pi)
xlabel('s [m]')
ylabel('\delta [\circ]')
subplot(4,1,2)
plot(z(1:j,6),z(1:j,4))
xlabel('s [m]')
ylabel('e_{y} [m]')
subplot(4,1,3)
plot(z(1:j,6),z(1:j,5)*180/pi)
xlabel('s [m]')
ylabel('e_{\psi} [\circ]')
subplot(4,1,4)
plot(z(1:j,6),params.v*ones(size(u(1:j))))
xlabel('s [m]')
ylabel('v [m/s]')