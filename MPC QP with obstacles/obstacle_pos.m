function pos = obstacle_pos(center,obs_size,paramsr)

pos = zeros(size(center,1),2);

for i = 1:size(center,1)
    
    r = sqrt(center(i,1)^2 + center(i,2)^2);
    
    if center(i,1)^2 + center(i,2)^2 ~= paramsr
       thetamod = acos(abs(center(i,1))/r);
       x = paramsr * cos(thetamod);
       if ~isequal(sign(center(i,1)), sign(x))
       x = x * sign(center(i,1));
       end
       center(i,1) = x;
       r = paramsr;
    end
    
    if center(i,1) >= 0 && center(i,2) > 0
        theta = acos(center(i,1)/r);
    elseif center(i,1) < 0 && center(i,2) > 0
        theta = pi - acos(abs(center(i,1))/r);
    elseif center(i,1) < 0 && center(i,2) < 0
        theta = pi + acos(abs(center(i,1))/r);
    elseif center(i,1) > 0 && center(i,2) < 0
        theta = 2*pi - acos(center(i,1)/r);
    end
    
    s = r * theta;
    pos(i,1) = s - obs_size(1)/2;
    pos(i,2) = s + obs_size(1)/2;
    
end