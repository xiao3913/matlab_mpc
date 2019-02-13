function plot_environment(z,params,varargin)
x = linspace(0,params.lane_length,100);
y = zeros(size(x));
%% parsing the parameters structure
vehicle_length      = params.l_f + params.l_r;
vehicle_width       = params.vehicle_width;
window_size         = params.window_size;
nstates             = params.nstates;
if nstates < 3
    error('The number of states cannot be less than 3!\n');
end
activate_obstacles  = params.activate_obstacles;
obstacle_centers    = params.obstacle_centers;
obstacle_size       = params.obstacle_size;
plot_full           = params.plot_full;

%% parsing the arguments
current_pos         = z(end,:);
if length(varargin) == 2
    z_horizon           = varargin{1};
    u_horizon           = varargin{2};
else
    z_horizon = [];
    u_horizon = [];
end
%% car description
car_x = [current_pos(1)-vehicle_length/2 current_pos(1)+vehicle_length/2 current_pos(1)+vehicle_length/2 current_pos(1)-vehicle_length/2];
car_y = [current_pos(2)+vehicle_width/2 current_pos(2)+vehicle_width/2 current_pos(2)-vehicle_width/2 current_pos(2)-vehicle_width/2];

%% window size limits
if plot_full == 1
    windowlimits_x = [0 x(end)];
    windowlimits_y = [-x(end)/3 x(end)/3];
elseif plot_full == 0
    if ~isempty(window_size) % if there are window_size limits
        windowlimits_x = [current_pos(1)-window_size current_pos(1)+window_size];
        windowlimits_y = [-lane_semiwidth lane_semiwidth];
    else % if there are no window_size limits then plot everything
        error('The window size cannot be an empty vector if plot_full is 0\n')
    end
else
    error('The flag plot_full must be set to 0 or 1\n')
end
windowlimits = [windowlimits_x,windowlimits_y];


%% define obstacles
if activate_obstacles == 1 % define obstacles
    obs_x = zeros(size(obstacle_centers,1),4); % edges x coordinates
    obs_y = zeros(size(obstacle_centers,1),4); % edges y coordinates
    for i = 1:size(obstacle_centers,1)
        xc = obstacle_centers(i,1); % obstacle center x coordinate
        xsize = obstacle_size(1)/2; % obstacle x size
        obs_x(i,:) = [xc-xsize, xc+xsize, xc+xsize, xc-xsize];
        
        yc = obstacle_centers(i,2); % obstacle center y coordinate
        ysize = obstacle_size(2)/2; % obstacle y size
        obs_y(i,:) = [yc+ysize, yc+ysize, yc-ysize, yc-ysize];
    end
    
elseif activate_obstacles == 0
    % do nothing
else
    error('The flag activate_obstacles must be set to 0 or 1\n')
end


figure(1);
%% plot simulation environment
hold off; plot(0,0); hold on;
% plot the car (x,y)
car_handle=patch(car_x,car_y,'white','EdgeColor','black');
rotate(car_handle,[0 0 1],rad2deg(current_pos(3)),[current_pos(1),current_pos(2) 0]);
plot(x,y,'--k','linewidth',3)
% plot the obstacles
if activate_obstacles == 1
    for i = 1:size(obstacle_centers,1)
        p = patch(obs_x(i,:),obs_y(i,:),'red','EdgeColor','red');
        rotate(p,[0,0,1],params.rot(i),[params.obstacle_centers(i,1),params.obstacle_centers(i,2),0])
    end
end
% plot the car path from the beggining of time
plot(z(:,1),z(:,2),'g');
% plot predicted vehicle states (x,y) if given
if ~isempty(z_horizon)
    plot(z_horizon(:,1),z_horizon(:,2),'-om');
end
axis(windowlimits)
xlabel('x');
ylabel('y');
drawnow;
end