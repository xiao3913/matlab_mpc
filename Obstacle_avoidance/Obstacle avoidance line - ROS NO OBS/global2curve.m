function [ey,epsi] = global2curve(params)

        psis = 0;                                % Heading angle of road [rad]
        Ys   = 0;                                % Reference point Ys [-]
        
        ey = (params.y0 - Ys)/cos(psis);              % Lateral deviation from road [m]
        epsi = params.psi0 - psis;                      % Deviation in heading angle w.r.t road [rad] 
end