function [ey,epsi] = global2curve(z)

        psis = 0;                                % Heading angle of road [rad]
        Ys   = 0;                                % Reference point Ys [-]
        
        ey = (z(2) - Ys)/cos(psis);              % Lateral deviation from road [m]
        epsi = z(3) - psis;                      % Deviation in heading angle w.r.t road [rad] 
end