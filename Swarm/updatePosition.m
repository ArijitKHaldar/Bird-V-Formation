% This function starts movement from present coordinates to given triangle
% coordinates on each iteration
function [Xtemp,Ytemp,VxTemp,VyTemp,j] = updatePosition(Xtemp,Ytemp,VxTemp,VyTemp,k1,k2,kv,N,b,c,Tstep,ScaleU,pos_target)

    j = 0; % No. of steps required to reach the formation coordinates

    while 1
        j = j+1;
        % Save the position and velocity of each agent at current n.
        %pos_begin=[X(n,:)' Y(n,:)']; % Forms a N X 2 array   % Never used this
        %vbar=mean([Vx(n,:)' Vy(n,:)']);

        % ErrorMatrix: 4xN, each column represents the error terms ([ep_x;ep_y;ev_x;ev_y]) of an agent.
        %ErrorMatrix=[X(n,:)'-pos_target(:,1) Y(n,:)'-pos_target(:,2) Vx(n,:)'-vbar(:,1) Vy(n,:)'-vbar(:,2)]'; % Not used anywhere !!

        EP_hat=[Xtemp(j,:); Ytemp(j,:)]; 
        % 2xN, [EP_hat(1,i); EP_hat(2,i)] is the position error of agent i with sensing error.
        % Note above that in 2d case, only the first two rows of 'dp' (which is 3xN) are used.

        % The 'for' loop below caculates the effect of the repel term on each agent (in 3 dimensions).
        for i=1:N
            Ediff=EP_hat(:,i)*ones(1,N)-EP_hat; % 2xN matrix. Column j (1<=j<=N) contains the error position difference of agent i and agent j in [x;y] direction, respectively.
            dist=sqrt(sum(Ediff.*Ediff)); % 1xN vector. The jth component is the norm of the error difference of agent i and j. It's equal to the distance from agent i to agent j; or .
            xrepel(i)=sum(b*exp(-dist.^2/c).*(Xtemp(j,i)-Xtemp(j,:)));
            yrepel(i)=sum(b*exp(-dist.^2/c).*(Ytemp(j,i)-Ytemp(j,:)));
        end
        % The 'for' loop below calculates the discrete gradient for each agent at current position.
%         A=zeros(N,2);
%         for i=1:N
%             NowJ=goalfunction0([Xtemp(j,i);Ytemp(j,i)],xgoal,w2) + obstaclefunction([Xtemp(j,i);Ytemp(j,i)],w1);
%             partial_x=VxTemp(j,i)*Tstep;
%             partial_y=VyTemp(j,i)*Tstep;
%             partialJx=goalfunction0([Xtemp(j,i)+partial_x;Ytemp(j,i)],xgoal,w2) + obstaclefunction([Xtemp(j,i)+partial_x;Ytemp(j,i)],w1) - NowJ;
%             partialJy=goalfunction0([Xtemp(j,i);Ytemp(j,i)+partial_y],xgoal,w2) + obstaclefunction([Xtemp(j,i);Ytemp(j,i)+partial_y],w1) - NowJ;        
%             A(i,:)=[partialJx/partial_x partialJy/partial_y];
%         end

        % Calculate the control input on two dimension x,y. Each u (i.e., ux, uy) is a 1xN vector.
        ux=-k1*(Xtemp(j,:)-pos_target(:,1)') - k2*(VxTemp(j,:)-mean(VxTemp(j,:))) - kv*VxTemp(j,:) + xrepel;
        uy=-k1*(Ytemp(j,:)-pos_target(:,2)') - k2*(VyTemp(j,:)-mean(VyTemp(j,:))) - kv*VyTemp(j,:) + yrepel;

        % Calculates the position and velocity in the next time step (Euler's method).
        Xtemp(j+1,:)=Xtemp(j,:)+VxTemp(j,:)*Tstep;
        Ytemp(j+1,:)=Ytemp(j,:)+VyTemp(j,:)*Tstep;
        VxTemp(j+1,:)=VxTemp(j,:) + ux*ScaleU*Tstep;
        VyTemp(j+1,:)=VyTemp(j,:) + uy*ScaleU*Tstep;
% Debugging code here
%         plot(Xtemp(j+1,:),Ytemp(j+1,:),'m*','LineWidth',2);
%         hold on;
%         axis([-5 30 -5 30]);
%         plot(pos_target(:,1),pos_target(:,2),'go');
%         hold off;
%         M(:,j)=getframe(gcf);
% Debugging code here
        % Stop when next N X 2 coordinates nearly same as previous N X 2 coordinates
        %result = sqrt(power(Xtemp(j+1,:)-Xtemp(j,:),2)+power(Ytemp(j+1,:)-Ytemp(j,:),2)); % If distance of movement between previous and current coordinates less than 0.00000001
        %expected = zeros(1,N);
        result = [Xtemp(j+1,:)',Ytemp(j+1,:)'];
        expected = pos_target;
        tolerance = 1e-6;
        if max(abs(result(:) - expected(:))) < tolerance % If already reached near formation coordinates, then stop
            j = j+1; % Saving total number of intermediate steps needed for reaching next formation coordinates correctly
            break;
        elseif j > 3000 % With higher number of agents, little corrections continue indefinitely
%             Xtemp(j+1,:) = pos_target(:,1)';
%             Ytemp(j+1,:) = pos_target(:,2)'; % Doing a dirty hack here to correct slight errors in limited time
            j = j+1;
            break;
        end
    end

end