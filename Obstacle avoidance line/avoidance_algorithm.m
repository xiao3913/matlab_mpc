function constraints = avoidance_algorithm(constraints,eps,z_pred,z_mpc,z,bound,params,i,k)

for j = 1:size(params.obstacle_centers,1)
    if z_pred(k,6) >= params.obstacle_centers(j,1) - params.obstacle_size(1)/2 && z_pred(k,6) <= params.obstacle_centers(j,1) + params.obstacle_size(1)/2
        
        if abs(params.obstacle_centers(j,2) + params.obstacle_size(2)/2) > abs(params.obstacle_centers(j,2) - params.obstacle_size(2)/2) % If center of obstacle is above zero line
            
            constraints = [constraints ,z_mpc(k,1) <= bound(j) + eps];        % Pass obstacle from below
            
        elseif abs(params.obstacle_centers(j,2) + params.obstacle_size(2)/2) < abs(params.obstacle_centers(j,2) - params.obstacle_size(2)/2) % If center of obstacle is below zero line
            
            constraints = [constraints ,z_mpc(k,1) >= bound(j) - eps];       % Pass obstacle from above
            
        elseif abs(params.obstacle_centers(j,2) + params.obstacle_size(2)/2) == abs(params.obstacle_centers(j,2) - params.obstacle_size(2)/2) % If center of obstacle is aligned with zero line
            
            if z(i,2) > 0
                
                constraints = [constraints ,z_mpc(k,1) >= bound(j) - eps];        % Pass obstacle from above
                
            elseif z(i,2) < 0
                
                constraints = [constraints ,z_mpc(k,1) <= bound(j) + eps];        % Pass obstacle from below
                
            end
        end
    end
end

end