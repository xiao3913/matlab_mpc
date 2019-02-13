function [x,y,psi] = curve2global2(params,z,j)

    theta = 1/params.r * z(j,6);        % Angle of revolution [rad]
    psis = theta + pi/2;                     % Heading angle of road [rad]
    Xs = params.r * cos(theta);              % Reference point Xs [-]
    Ys = params.r * sin(theta);              % Reference point Ys [-]

    x = Xs - z(j,4) * sin(psis);     % Global X coordinate [-]
    y = Ys + z(j,4) * cos(psis);     % Global Y coordinate [-]
    psi = psis + z(j,5);             % Global headin angle [rad]

end