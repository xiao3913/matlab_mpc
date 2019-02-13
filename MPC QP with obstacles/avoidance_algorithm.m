% Obstacle avoidance algorithm
function [A,b,H] = avoidance_algorithm(z_pred,params,ind,indpsi,inddelta,A,b,k,pos,H)

centers = params.obstacle_centers;
sizes   = params.obstacle_size;
fri = 0;

for i = 1:size(centers,1)
    
    if z_pred(k,6) >= pos(i,1) && z_pred(k,6) <= pos(i,2)       % Detect a collision
        r = sqrt(centers(i,1)^2+centers(i,2)^2);                % Shortest distance from origo to an obstacle [m]
        if r-sizes(2)/2>params.r || r+sizes(2)/2<params.r       % Is an obstacle out of range (inside or outside circle)?
            % Do nothing
            fri = 1;
            
        elseif r > params.r && fri ~=1                               % If obstacle lies outside of circle
            b(2*params.horizon+k) = -1*(sizes(2)/2-(r-params.r));    % Pass from inside of circle
            A(2*params.horizon+k,ind(k)) = -1;
            A(2*params.horizon+k,end) = -1;
            
            H(ind(k),ind(k)) = 100;            % Adjusted weight for lateral deviation [-]
            H(indpsi(k),indpsi(k)) = 0.01;    % Adjusted weight for deviation in heading angle [-]
            H(inddelta(k),inddelta(k)) = 0.1;  % Adjusted weight for control input [-]
            H(end) = 1e20;                    % Adjusted weight for slack variable [-]
            
        elseif r < params.r && fri ~=1                               % If obstacle lies inside of circle
            b(2*params.horizon+k) = -abs(sizes(2)/2+r-params.r);     % Pass from outside of circle
            A(2*params.horizon+k,ind(k)) = 1;
            A(2*params.horizon+k,end) = -1;
            
            H(ind(k),ind(k)) = 100;            % Adjusted weight for lateral deviation [-]
            H(indpsi(k),indpsi(k)) = 0.01;    % Adjusted weight for deviation in heading angle [-]
            H(inddelta(k),inddelta(k)) = 0.1;  % Adjusted weight for control input [-]
            H(end) = 1e20;                    % Adjusted weight for slack variable [-]
            
        elseif r == params.r
            
            if sqrt(centers(i+1,1)^2+centers(i+1,1)^2) > params.r && fri ~=1 && i <= size(pos,1) % If next obstacle lies outside of circle
                b(2*params.horizon+k) = -1*(sizes(2)/2-(r-params.r));                            % Pass from inside of circle
                A(2*params.horizon+k,ind(k)) = -1;
                A(2*params.horizon+k,end) = -1;
                
                H(ind(k),ind(k)) = 100;            % Adjusted weight for lateral deviation [-]
                H(indpsi(k),indpsi(k)) = 0.01;    % Adjusted weight for deviation in heading angle [-]
                H(inddelta(k),inddelta(k)) = 0.1;  % Adjusted weight for control input [-]
                H(end) = 1e20;                    % Adjusted weight for slack variable [-]
                
            elseif sqrt(centers(i+1,1)^2+centers(i+1,1)^2) < params.r && fri ~=1 && i <= size(pos,1) % If next obstacle lies inside of circle
                b(2*params.horizon+k) = -abs(sizes(2)/2+r-params.r);                                 % Pass from outside of circle
                A(2*params.horizon+k,ind(k)) = 1;
                A(2*params.horizon+k,end) = -1;
                
                H(ind(k),ind(k)) = 100;            % Adjusted weight for lateral deviation [-]
                H(indpsi(k),indpsi(k)) = 0.01;    % Adjusted weight for deviation in heading angle [-]
                H(inddelta(k),inddelta(k)) = 0.1;  % Adjusted weight for control input [-]
                H(end) = 1e20;                    % Adjusted weight for slack variable [-]
                
            else
                b(2*params.horizon+k) = -1*(sizes(2)/2-(r-params.r));    % Pass from inside of circle
                A(2*params.horizon+k,ind(k)) = -1;
                A(2*params.horizon+k,end) = -1;
                
                H(ind(k),ind(k)) = 100;            % Adjusted weight for lateral deviation [-]
                H(indpsi(k),indpsi(k)) = 0.01;    % Adjusted weight for deviation in heading angle [-]
                H(inddelta(k),inddelta(k)) = 0.1;  % Adjusted weight for control input [-]
                H(end) = 1e20;                    % Adjusted weight for slack variable [-]
            end
            
        end
    end
    fri = 0;
end