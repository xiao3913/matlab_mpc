function [x,y,psi] = curve2global(params,z_pred)

x = zeros(length(params.horizon));
y = zeros(length(params.horizon));
psi = zeros(length(params.horizon));

for w = 1:params.horizon
    
        psis = 0;                                % Heading angle of road [rad]
        Xs = z_pred(w,6);                        % Reference point Xs [-]
        Ys = 0;                                  % Reference point Ys [-]
        
        x(w) = Xs - z_pred(w,4) * sin(psis);     % Global X coordinate [-]
        y(w) = Ys + z_pred(w,4) * cos(psis);     % Global Y coordinate [-]
        psi(w) = psis + z_pred(w,5);             % Global heading angle [rad]
end
end