% Model predictive control (MPC) as quadratic programming
clear, clc, close

% Model parameters
params.delta_delta_max              = pi/60;                          % Maximum rate of change in steering angle [per 50 ms]
params.delta_max                    = pi/6;                           % Maximum steering angle [-]
params.l_f                          = 0.16;                           % Distance between C.O.G and front axle [m]
params.l_r                          = 0.16;                           % Distance between C.O.G and rear axle [m]
params.l                            = params.l_f+params.l_r;          % Wheelbase [m]
params.vehicle_width                = 0.25;                           % Width of vehicle [m]
params.vehicle_length               = 0.48;                           % Length of vehicle [m]
params.Ts                           = 0.05;                           % Sampling time [s]
params.nstates                      = 4;                              % Number of states [-]
params.ninputs                      = 1;                              % Number of inputs [-]
params.rou                          = inf;                            % Turning Radius [m]

% environment parameters
params.activate_obstacles           = 0;                                                          % 0 if there are no obstacles, 1 otherwise [-]
params.obstacle_centers             = [];                                                         % x and y coordinates of the obstacles
params.rot                          = [];                                                         % Rotation of obstacles [degree]

params.obstacle_size                = [];                                                         % Size of the obstacles (x,y) [m]
params.lane_length                  = 5.106;                                                      % Length of road [m]
params.window_size                  = 10;                             % Plot window size for the during simulation, centered at the current position (x,y) of the vehicle
params.plot_full                    = 1;                              % 0 for plotting only the window size, 1 for plotting from 0 to end of track


% Control parameters
params.horizon                      = 25;                              % Prediction horizon [-]
q1                                  = 0.8;                            % Weight for lateral deviation [-]
q2                                  = 0.01;                           % Weight for deviation of heading angle [-]
q3                                  = 0;                              % Weight for distance covered [-]
q4                                  = 1E3;                            % Weight for slack variable [-]
q5                                  = 0.1;                          % Weight for control input [-]
q6                                  = 0;                          % Weight for change of rate of control input [-]
Q                                   = [q1 q2 q3 q5];

% Simulation parameters
params.x0                           = 0;                              % Initial x coordinate [-]
params.y0                           = -0.4;                              % Initial y coordinate [-]
params.psi                          = 0;                              % Initial heading angle [-]
params.ey0                          = 0;                              % Initial lateral deviation from centerline [m]
params.epsi                         = 0;                              % Initial deviation from heading angle of centerline [-]
params.s0                           = 0;                              % Initial distance covered along centerline [m]
params.N_max                        = 1000;                           % Maximum number of simulation steps [-]
params.z0                           = [-0.4; 0; 0; 0];                   % Initial state [-]

% Define matrices for QP
% Construct matrix H
H                                                                                       = zeros(5*params.horizon);
H(1:(params.horizon*length(Q)),1:(params.horizon*length(Q)))                            = kron(eye(params.horizon),diag(Q));
H                                                                                       = [H; zeros(1,length(H))];
H                                                                                       = [H zeros(size(H,1),1)];
H(end)                                                                                  = q4;
H(params.horizon*length(Q)+1:size(H,1)-1,params.horizon*length(Q)+1:size(H,1)-1)        = kron(eye(params.horizon),diag(q6));
H                                                                                       = 2*H;


% MPC algorithm
zt                          = params.z0;                                     % Initial state
f                           = [];
lb                          = [];
ub                          = [];
x0                          = [];
options                     = optimset('largescale','off','Display','off');
t                           = zeros(params.N_max,1);                        % Time to solve [s]
z                           = zeros(params.N_max,params.nstates+3);         % Full state vector
z_pred                      = zeros(params.horizon,params.nstates+3);       % Predicted states
u                           = zeros(params.N_max,params.ninputs);           % Steering angle [radians]
u_pred                      = zeros(params.horizon,params.ninputs);         % Predicted steering angle [radians]
z(1,:)                      = [zt(3) zt(1) zt(2) zt(1) zt(2) zt(3) zt(4)];  % Initial full state vector
j                           = 1;
ind                         = 1:params.nstates:params.horizon*params.nstates;                % Vector containing indicis for constraints on ey and adjusted weight on ey [-]
indpsi                      = 2:params.nstates:params.nstates*params.horizon - 1;            % Vector containing indicis for adjusted weight on epsi [-]
inddelta                    = params.nstates*params.horizon+1:params.ninputs:size(H,1)-1;    % Vector containing indicis for control input [-]
variance                    = 1E-5;                                                          % Variance for white noise

while z(j,6) <=  params.lane_length
    noisev                              = randn(1,1) * sqrt(variance);                           % Generate white noise with specific variance for velocity
    params.v                            = 0.5 + noisev;                                          % Velocity [m/s]
    params.deltas                       = params.Ts*params.v;                                    % Dicretization step [m]
    
    params.AA                           = [1 params.deltas 0; 0 1 0; params.deltas/params.rou 0 1];   % State prediction matrix
    params.BB                           = [0; params.deltas/params.l; 0];                             % Output prediction matrix
    params.CC                           = [0; -params.deltas/params.rou; params.deltas];              % Constant vector for system dynamics
    
    params.A                            = [params.AA                  params.BB;                      % Augmented state prediction matrix zeros(size(params.AA,1),1)
                                           zeros(1,size(params.AA,1)) ones(1,1)];
    params.B                            = [params.BB; ones(1,1)];                                     % Augmented output prediction matrix
    params.C                            = [params.CC; zeros(1,1)];                                    % Augmented constant vector for system dynamics
    
    % Construct matrix AA that defines beq in the MPC - loop
    AA = zeros(size(params.A,1)*params.horizon,size(params.A,1));
    
    % Construct constant vector C
    C = zeros(params.nstates*params.horizon,1);
    
    % Construct matrix Aeq that defines the feasibility of states
    Aeq = zeros(params.nstates*params.horizon,5*params.horizon);
    Aeq(1:(params.nstates*params.horizon),1:(params.nstates*params.horizon)) = eye(params.nstates*params.horizon);
    con = zeros(params.nstates*params.horizon+params.nstates,1);
    con(1:length(params.C)) = params.C;
    
    % Define constraint on steering angle, ey and on slack variable
    A = zeros(2*params.horizon,5*params.horizon);
    A(1:params.horizon,end-params.horizon+1:end) = eye(params.horizon);                       % Upper bound on rate of change of steering angle
    A(params.horizon+1:2*params.horizon,end-params.horizon+1:end) = -eye(params.horizon);     % Lower bound on rate of change of steering angle
    A = [A zeros(size(A,1),1)];
    A = [A;zeros(1,size(A,2))];
    A2 = zeros(3*params.horizon,size(A,2));                                                   % Add rows for constraints on state e_y
    A2(end) = -1;                                                                             % Non-negativity constraint on slack variable
    
    A = [A;A2];
    b = zeros(size(A,1),1);
    b(1:2*params.horizon) = params.delta_delta_max*ones(2*params.horizon,1);                  % Constraints on the upper and lower bounds of rate of change of steering angle
    b(3*params.horizon+1:end-1) = params.delta_max;                                           % Constraints on the upper and lower bounds of maximum steering angle
    inddeltastate = 4:4:params.horizon*4;
    
    for i = 1:params.horizon
        % Construct matrix AA that defines beq in the MPC - loop
        AA(params.nstates*(i-1)+1:params.nstates*(i-1)+4,:) = params.A^i;
        
        % Construct the constant vector C
        C(params.nstates*(i-1)+1:params.nstates*(i-1)+4) = con(params.nstates*(i-1)+1:params.nstates*(i-1)+4);
        con(params.nstates*(i-1)+5:params.nstates*(i-1)+8) = con(params.nstates*(i-1)+1:params.nstates*(i-1)+4)+params.A^(i)*params.C;
        
        % Construct matrix Aeq that defines the feasibility of states
        Aeq(params.nstates*(i-1)+1:params.nstates*params.horizon,end-params.horizon+1:end-i+1) = Aeq(params.nstates*(i-1)+1:params.nstates*params.horizon,end-params.horizon+1:end-i+1)+kron(eye(params.horizon-i+1),-params.A^(i-1)*params.B);
        
        % Construct matrix A for inequality constraints
        A(3*params.horizon+i,inddeltastate(i)) = 1;
        A(3*params.horizon+params.horizon+i,inddeltastate(i)) = -1;
        A(3*params.horizon+i,end) = -1;
        A(3*params.horizon+params.horizon+i,end) = -1;
    end
    Aeq = [Aeq zeros(size(Aeq,1),1)];
    
    noisey          = randn(1,1) * sqrt(variance);                           % Generate white noise with specific variance for ey
    noisepsi        = randn(1,1) * sqrt(variance);                           % Generate white noise with specific variance for epsi
    beq             = AA*z(j,4:7)'+C;                                        % The matrix AA defines how the last measured state determines the r.h.s in the equality constraint
    x               = quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);            % Solution obtained from QUADPROG
    u(j)            = x(params.nstates*params.horizon+1);                    % Change of rate of steering angle [-]
    u_pred          = x(4:4:params.nstates*params.horizon);                  % Predicted change of rate of steering angle [-]
    z(j+1,7)        = x(4);                                                  % Steering angle [-]
    z(j+1,4:6)      = nonlinear_dynamics(z(j,4:7),params);                   % Nonlinear dynamics x, y and psi
    z(j+1,1)        = z(j+1,6);                                              % x coordinate
    z(j+1,2)        = z(j+1,4);                                              % y coordinate
    z(j+1,3)        = z(j+1,5);                                              % Heading angle [-]
    z_pred(:,7)     = x(params.nstates*params.horizon+1:end-1);              % Predicted steering angle [-]
    z_pred(:,4)     = x(1:4:params.nstates*params.horizon-2);                % Predicted lateral deviation [m]
    z_pred(:,5)     = x(2:4:params.nstates*params.horizon-1);                % Predicted deviation of heading angle w.r.t centerline [radians]
    z_pred(:,6)     = x(3:4:params.nstates*params.horizon);                  % Predicted distance covered [m]
    z_pred(:,1)     = z_pred(:,6);                                           % Predicted x coordinate
    z_pred(:,2)     = z_pred(:,4);                                           % Predicted y coordinate
    z_pred(:,3)     = z_pred(:,5);                                           % Predicted heading angle
    z(j+1,4)        = z(j+1,4) + noisey;                                     % Add white noise to ey
    z(j+1,5)        = z(j+1,5) + noisepsi;                                   % Add white noise to epsi
    plot_environment(z(1:j,:),params,z_pred,z_pred(:,7));
    t(j)            = toc;
    j               = j+1;
end
plot_environment(z(1:j,:),params);
figure()
subplot(4,1,1)
plot(z(1:j,6),z(1:j,7)*180/pi)
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