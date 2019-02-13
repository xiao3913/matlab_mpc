function [x,y,psi] = curve2global(params,z_pred)

x = zeros(length(params.horizon));
y = zeros(length(params.horizon));
psi = zeros(length(params.horizon));

for w = 1:params.horizon
    
        theta = 1/params.r * z_pred(w,6);        % Angle of revolution [rad]
        psis = theta + pi/2;                     % Heading angle of road [rad]
        Xs = params.r * cos(theta);              % Reference point Xs [-]
        Ys = params.r * sin(theta);              % Reference point Ys [-]
        
        x(w) = Xs - z_pred(w,4) * sin(psis);     % Global X coordinate [-]
        y(w) = Ys + z_pred(w,4) * cos(psis);     % Global Y coordinate [-]
        psi(w) = psis + z_pred(w,5);             % Global headin angle [rad]
end
end