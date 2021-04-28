function dis = ur5RRcontrol(gdesired, K, ur5)
% resolved rate control method
    time_step = 0.05;
    position_threshold = 2*1e-2; % m
    angle_threshold = 5*pi/180; % rad
    minimum_manipulability = 0.008;
    
    while 1
        
       
        currentq = ur5.get_current_joints(); % current joint angles
        gst = ur5FwdKin(currentq);  % end effecotr's current pose
        JB = ur5BodyJacobian(currentq); % current body jacobian
        
        manipuability_sigmamin = manipulability(JB, 'sigmamin'); % sigmamin manipulability
        
        % check if the body jacobian matrix is singular
        rank_JB = rank(JB);
        if(rank_JB ~=6 | manipuability_sigmamin < minimum_manipulability)
            dis = -1;
            error('finalerror:-1');
            return;
        end
        
        
        JB_inv = JB^(-1);
        
        % update joint angles
        [nexq, xi] = RRC_OneStep(currentq, K, time_step, JB_inv, gdesired, gst);
        ur5.move_joints(nexq, time_step);
        
        % distance and angle from the target pose
        distance2goal = norm(xi(1:3));
        angle2goal = norm(xi(4:6));
        
        % check if target pose is reached
        if((distance2goal < position_threshold) & (angle2goal < angle_threshold))
            dis = distance2goal * 100; % convert m to cm
            disp('Reached goal configuration');
            return;
        end
        
        pause(time_step);
    end
end

function [nextq, xi] = RRC_OneStep(currentq, k, time_step, JB_inv, gst_star, gst)
% one step of resolved rate control law

    g_tstar_t = (gst_star^(-1))*gst; % transformation matrix from target to current pose
    xi = getXi(g_tstar_t);
    dv = JB_inv*xi;
    
    % make sure the speed of joints will not exceed the maxmum speed limit
    maxmum_speed = pi/2;
    k_max = (2/3)*maxmum_speed / max(abs(dv));
    
    if k > k_max
        k = k_max;
    end
    
    nextq = currentq - k*time_step*dv;
    
end