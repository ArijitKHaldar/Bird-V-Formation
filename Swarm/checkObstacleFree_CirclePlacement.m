% This function is used to check if the sensed circle hits one or more obstacles or not, if yes, push the circle away from the obstacles, till a safe place is
% reached. Returns the new circle details with or without modifications, as required
function [P,Circle_Co,cirCenter] = checkObstacleFree_CirclePlacement(obstacle,P)
    in = 1;
    on = 1;
    while any(in)||any(on) % Iterate until circle not touching/on any obstacle
        loc_spline = [P(1,1) P(1,2)];
        cirCenter = loc_spline; % This has the coordinates of center of circle after fitting is done
        [X_dash,Y_dash] = findCirclePoints(P);
        Circle_Co = [X_dash,Y_dash]; % This has 50 coordinates on the circumference of the circle
        [in,on] = inpolygon(obstacle(:,1),obstacle(:,2),Circle_Co(:,1),Circle_Co(:,2));
        if any(in)||any(on)
            for i=1:size(obstacle(:,1))
                [checkIn,checkEdge] = inpolygon(obstacle(i,1),obstacle(i,2),Circle_Co(:,1),Circle_Co(:,2)); % check hit of any point with i-th obstacle
                if any(checkIn)||any(checkEdge)
                   obsToCircumference(1,:) = sqrt(power((obstacle(i,1)-Circle_Co(:,1)),2)+power((obstacle(i,2)-Circle_Co(:,2)),2));
                   [~,index] = min(obsToCircumference);
%                    P(1,1) = P(1,1)+1*((obstacle(i,1)-Circle_Co(index,1))/abs((obstacle(i,1)-Circle_Co(index,1))));
%                    P(1,2) = P(1,2)+1*((obstacle(i,2)-Circle_Co(index,2))/abs((obstacle(i,2)-Circle_Co(index,2))));
%                    P(1,1) = P(1,1)+1;
                   P(1,2) = P(1,2)-1;
                   break;
                end
            end
        end
    end
end