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

params.A                            = [1 params.deltas 0; 0 1 0; params.deltas/params.rou 0 1];   % State prediction matrix
params.B                            = [0; params.deltas/params.l; 0];                             % Output prediction matrix
params.C                            = [0; -params.deltas/params.rou; params.deltas];              % Constant vector for system dynamics

% environment parameters
params.activate_obstacles           = 1;                                                          % 0 if there are no obstacles, 1 otherwise [-]
params.obstacle_centers             = [(params.r-0.04)*cos(pi/4) (params.r-0.04)*sin(pi/4);
                                       0 params.r;
                                       -(params.r+0.03)*cos(pi/4) (params.r+0.03)*sin(pi/4);
                                       -(params.r)*cos(pi/3) -(params.r)*sin(pi/3);
                                       (params.r+0.015)*sin(pi/5) -(params.r+0.015)*cos(pi/5)];                      % X and Y coordinates of the obstacles
params.rot                          = [135 0 45 146.9203 36];                                                        % Rotation of obstacles [degree]
params.obstacle_size                = [0.2 0.1];                                                                     % Size of the obstacles (l,w) [m]
pos                                 = obstacle_pos(params.obstacle_centers,params.obstacle_size,params.r);           % Calculates position of obstacles along path
params.window_size                  = 10;                                                                  % Plot window size for the during simulation, centered at the current position (x,y) of the vehicle
params.plot_full                    = 1;                                                                   % 0 for plotting only the window size, 1 for plotting from 0 to end of track


% Control parameters
params.horizon                      = 20;                             % Prediction horizon [-]
q1                                  = 0.8;                             % Weight for lateral deviation [-]
q2                                  = 0.5;                              % Weight for deviation of heading angle [-]
q3                                  = 0;                              % Weight for distance covered [-]
q4                                  = 0;                              % Weight for slack variable [-]
q5                                  = 0.1;                            % Weight for input [-]                               
Q                                   = [q1 q2 q3];

% Simulation parameters
params.x0                           = 0;                              % Initial x coordinate
params.y0                           = 0;                              % Initial y coordinate
params.psi                          = 0;                              % Initial heading angle [-]
params.ey0                          = 0;                              % Initial lateral deviation from centerline [m]
params.epsi                         = 0;                              % Initial deviation from heading angle of centerline [-]
params.s0                           = 0;                              % Initial distance covered along centerline [m]
params.N_max                        = 500;                            % Maximum number of simulation steps []
params.z0                           = [0; 0; 0];                    % Initial state []

% Define matrices for QP
% Construct matrix H
H                                                                                       = zeros(4*params.horizon);
H(1:(params.horizon*length(Q)),1:(params.horizon*length(Q)))                            = kron(eye(params.horizon),diag(Q));
H                                                                                       = [H; zeros(1,length(H))];
H                                                                                       = [H zeros(size(H,1),1)];
H(end)                                                                                  = q4;
H(params.horizon*length(Q)+1:size(H,1)-1,params.horizon*length(Q)+1:size(H,1)-1)        = q5*eye(params.horizon);    
H                                                                                       = 2*H;
Hopt                                                                                    = H;          % Objective function for optimal path tracking

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
Aeq = [Aeq zeros(size(Aeq,1),1)];                                                         % Add an extra column of zeros for slack variable

% Define constraint on steering angle, ey and on slack variable
A = zeros(2*params.horizon,4*params.horizon);
A(1:params.horizon,end-params.horizon+1:end) = eye(params.horizon);                       % Upper bound on steering angle
A(params.horizon+1:2*params.horizon,end-params.horizon+1:end) = -eye(params.horizon);     % Lower bound on steering angle                                           
A = [A zeros(size(A,1),1)];
A = [A;zeros(1,size(A,2))];
A2 = zeros(params.horizon,size(A,2));                                                     % Add rows for constraints on state e_y
A2(end) = -1;                                                                             % Rewrite constraint on slack variable

A = [A;A2];
b = zeros(size(A,1),1);
b(1:2*params.horizon) = params.delta_max*ones(2*params.horizon,1);                        % Constraints on the upper and lower bounds of steering angle

% MPC algorithm
zt                          = params.z0;                                     % Initial state
f                           = [];
x0                          = [];
ub                          = [];
lb                          = [];
options                     = optimset('largescale','off','Display','off');
t                           = zeros(params.N_max,1);                         % Time to solve [s]
z                           = zeros(params.N_max,params.nstates+3);          % Full state vector
z_pred                      = zeros(params.horizon,params.nstates+3);        % Predicted states
u                           = zeros(params.N_max,params.ninputs);            % Steering angle [radians]
u_pred                      = zeros(params.horizon,params.ninputs);          % Predicted steering angle [radians]
z(1,:)                      = [params.r 0 pi/2 zt(1) zt(2) zt(3)];         % Initial full state vector
j                           = 1;
ind                         = 1:params.nstates:params.horizon*params.nstates;% Vector containing indicis
indpsi                      = 2:params.nstates:params.nstates*params.horizon - 1;      % Vector containing indicis for adjusted weight on epsi [-]
inddelta                    = params.nstates*params.horizon:params.ninputs:size(H,1);  % Vector containing indicis for control input [-]        
H                           = Hopt;                                                    % Start by optimizing w.r.t path
variance                    = 1E-6;

while z(j,6) <= 2*pi*params.r
    tic
    noisex          = randn(1,1) * sqrt(variance);                           % Generate white noise with a specific variance and add to x, y and psi
    noisey          = randn(1,1) * sqrt(variance);
    zt(1)           = zt(1) + noisex;
    zt(2)           = zt(2) + noisey;
    if j > 1
    for k = 1:params.horizon
        [A,b,H] = avoidance_algorithm(z_pred,params,ind,indpsi,inddelta,A,b,k,pos,H);
    end
    end
    beq             = AA*zt+C;                                               % The matrix AA defines how the last measured state determines the r.h.s in the equality constraint
    x               = quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);            % Solution obtained from QUADPROG
    ut              = x(params.nstates*params.horizon+1);                    % First calculated steering angle [radians]
    zt              = params.A * zt + params.B * ut + params.C;              % Calculation of next state
    u(j)            = x(params.nstates*params.horizon+1);                    % Steering angle [radians]
    u_pred          = x(params.nstates*params.horizon+1:end);                % Predicted steering angle [radians]
    z(j+1,4:6)      = nonlinear_dynamics(zt,u,params,j);                     % Nonlinear dynamics
    z_pred(:,4)     = x(1:3:params.nstates*params.horizon-2);                % Predicted lateral deviation [m]
    z_pred(:,5)     = x(2:3:params.nstates*params.horizon-1);                % Predicted deviation of heading angle w.r.t centerline [radians]
    z_pred(:,6)     = x(3:3:params.nstates*params.horizon);                  % Predicted distance covered [m]
    [z_pred(1:params.horizon,1),z_pred(1:params.horizon,2),z_pred(1:params.horizon,3)] = curve2global(params,z_pred);       % Calculates global x, y and psi from curvelinear coordinates
    [z(j+1,1), z(j+1,2), z(j+1,3)] = curve2global2(params,z,j);                                                                              % Converts states from nonlinear model to global coordinates
    plot_environment(z(1:j,:),params,z_pred,u_pred);
    A(2*params.horizon+1:end-1,:) = 0;
    b(2*params.horizon+1:end-1) = 0;
    H               = Hopt;                                                  % Reset objective function to optimal path
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