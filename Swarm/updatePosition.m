% This function starts movement from present coordinates to given triangle coordinates on each iteration
function [Xtemp,Ytemp,VxTemp,VyTemp,j] = updatePosition(Xtemp,Ytemp,VxTemp,VyTemp,k1,k2,kv,kf,w1,w2,N,b,c,Tstep,~,xgoal,obstacle,pos_target)
    ScaleU = 180;
    for j=1:299

        EP_hat=[Xtemp(j,:); Ytemp(j,:)]; 

        % The 'for' loop below caculates the effect of the repel term on each agent (in 3 dimensions).
        for i=1:N
            Ediff=EP_hat(:,i)*ones(1,N)-EP_hat; % 2xN matrix. Column j (1<=j<=N) contains the error position difference of agent i and agent j in [x;y] direction, respectively.
            dist=sqrt(sum(Ediff.*Ediff)); % 1xN vector. The jth component is the norm of the error difference of agent i and j. It's equal to the distance from agent i to agent j; or .
            xrepel(i)=sum(b*exp(-dist.^2/c).*(Xtemp(j,i)-Xtemp(j,:)));
            yrepel(i)=sum(b*exp(-dist.^2/c).*(Ytemp(j,i)-Ytemp(j,:)));
        end
        % The 'for' loop below calculates the discrete gradient for each agent at current position.
        A=zeros(N,2);
        for i=1:N
            NowJ=goalfunction0([Xtemp(j,i);Ytemp(j,i)],xgoal,w2) + obstaclefunction([Xtemp(j,i);Ytemp(j,i)],w1,obstacle);
            partial_x=VxTemp(j,i)*Tstep;
            partial_y=VyTemp(j,i)*Tstep;
            partialJx=goalfunction0([Xtemp(j,i)+partial_x;Ytemp(j,i)],xgoal,w2) + obstaclefunction([Xtemp(j,i)+partial_x;Ytemp(j,i)],w1,obstacle) - NowJ;
            partialJy=goalfunction0([Xtemp(j,i);Ytemp(j,i)+partial_y],xgoal,w2) + obstaclefunction([Xtemp(j,i);Ytemp(j,i)+partial_y],w1,obstacle) - NowJ;        
            A(i,:)=[partialJx/partial_x partialJy/partial_y];
        end

        % Calculate the control input on two dimension x,y. Each u (i.e., ux, uy) is a 1xN vector.
        ux=-k1*(Xtemp(j,:)-pos_target(:,1)') - k2*(VxTemp(j,:)-mean(VxTemp(j,:))) - kv*VxTemp(j,:) + xrepel - kf*(A(:,1)');
        uy=-k1*(Ytemp(j,:)-pos_target(:,2)') - k2*(VyTemp(j,:)-mean(VyTemp(j,:))) - kv*VyTemp(j,:) + yrepel - kf*(A(:,2)');

        % Calculates the position and velocity in the next time step (Euler's method).
        Xtemp(j+1,:)=Xtemp(j,:)+VxTemp(j,:)*Tstep;
        Ytemp(j+1,:)=Ytemp(j,:)+VyTemp(j,:)*Tstep;
        VxTemp(j+1,:)=VxTemp(j,:) + ux*ScaleU*Tstep;
        VyTemp(j+1,:)=VyTemp(j,:) + uy*ScaleU*Tstep;
        % Debugging code here
%                 plot(Xtemp(j+1,:),Ytemp(j+1,:),'m*','LineWidth',2);
%                 hold on;
%                 axis([-5 30 -5 30]);
%                 plot(pos_target(:,1),pos_target(:,2),'go');
%                 hold off;
%                 M(:,j)=getframe(gcf);
        % Debugging code here
    end
    j=j+1; % Because we are using the j+1-th position of the array, so, readjusting the total loop size
end