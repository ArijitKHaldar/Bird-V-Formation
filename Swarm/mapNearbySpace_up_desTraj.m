% For all n ~= 1, i.e., for all iterations other than first
function [X,Y,Vx,Vy]=mapNearbySpace_up_desTraj(k1,k2,kv,kf,b,c,X_1,Y_1,Vx_1,Vy_1,xgoal,obstacle)

    Tstep=0.01;
    N=100;

    % Goal position of vehicle
    % xgoal=[80; 80];
    w1=120; 		% Set weighting factors on the goal function and obstacle function
                    % w1=120 and w2=0.1 give good result ! 03/11/03
    w2=1.0000e-01;

%     Dmax=10; % The magnitude of the noise. Since we use uniform noise of Matlab here, Dmax=1.
    ScaleU=10; % This is used to change the magnitude of the control input ux and uy.

    % Initialization of position and velocity
    X(1,1:N)=X_1(end,:); 
    Y(1,1:N)=Y_1(end,:); 
    Vx(1,1:N)=Vx_1(end,:); 
    Vy(1,1:N)=Vy_1(end,:); 

%     count1=1;

    for count1=1:49

%         xbar=mean([X(end,:)' Y(end,:)']);       % 2x1 vector of means in X and Y dimensions
%         vbar=mean([Vx(end,:)' Vy(end,:)']);      % and for velocity also

        %ErrorMatrix=[X(end,:)-xbar(1); Y(end,:)-xbar(2); Vx(end,:)-vbar(1); Vy(end,:)-vbar(2)];

        EP_hat=[X(end,:); Y(end,:)];

        for i=1:N
            Ediff=EP_hat(:,i)*ones(1,N)-EP_hat; % 2xN matrix. Column j (1<=j<=N) contains the error position difference of agent i and agent j in [x;y] direction, respectively.
            dist=sqrt(sum(Ediff.*Ediff)); % 1xN vector. The jth component is the norm of the error difference of agent i and j. It's equal to the distance from agent i to agent j; or .
            xrepel(i)=sum(b*exp(-dist.^2/c).*(X(end,i)-X(end,:)));
            yrepel(i)=sum(b*exp(-dist.^2/c).*(Y(end,i)-Y(end,:)));
        end
        % The 'for' loop below calculates the discrete gradient for each agent at current position.
        A=[];
        for i=1:N
    %         NowJ=goalfunction0([X(end,i);Y(end,i)],xgoal,w2) + obstaclefunctionComplex([X(end,i);Y(end,i)],w1);
            NowJ=goalfunction0([X(end,i);Y(end,i)],xgoal,w2) + obstaclefunction([X(end,i);Y(end,i)],w1,obstacle);
            partial_x=Vx(end,i)*Tstep;
            partial_y=Vy(end,i)*Tstep;
    %         partialJx=goalfunction0([X(end,i)+partial_x;Y(end,i)],xgoal,w2) + obstaclefunctionComplex([X(end,i)+partial_x;Y(end,i)],w1) - NowJ;
    %         partialJy=goalfunction0([X(end,i);Y(end,i)+partial_y],xgoal,w2) + obstaclefunctionComplex([X(end,i);Y(end,i)+partial_y],w1) - NowJ;        
            partialJx=goalfunction0([X(end,i)+partial_x;Y(end,i)],xgoal,w2) + obstaclefunction([X(end,i)+partial_x;Y(end,i)],w1,obstacle) - NowJ;
            partialJy=goalfunction0([X(end,i);Y(end,i)+partial_y],xgoal,w2) + obstaclefunction([X(end,i);Y(end,i)+partial_y],w1,obstacle) - NowJ;        
            A(i,:)=[partialJx/partial_x partialJy/partial_y];
        end

        % Calculate the control input on two dimension x,y. Each u (i.e., ux, uy) is a 1xN vector.
        ux(1,:)=-k1*(X(end,:)-mean(X(end,:))) - k2*(Vx(end,:)-mean(Vx(end,:))) - kv*Vx(end,:) + 1.25*xrepel - kf*(A(:,1)');
        uy(1,:)=-k1*(Y(end,:)-mean(Y(end,:))) - k2*(Vy(end,:)-mean(Vy(end,:))) - kv*Vy(end,:) + yrepel - kf*(A(:,2)');
        %uz=-k1*(Z(n,:)-mean(Z(n,:))-dp(3,:)) - k2*(Vz(n,:)-mean(Vz(n,:))-dv(3,:)) - kv*Vz(n,:) + zrepel - kf*(A(:,3)'-df(3,:)); % For 3-d case.

        % Calculates the position and velocity in the next time step (Euler's method).
        X(end+1,:)=X(end,:)+Vx(end,:)*Tstep;
        Y(end+1,:)=Y(end,:)+Vy(end,:)*Tstep;
        %Z(n+1,:)=Z(n,:)+Vz(n,:)*Tstep; % For 3-d case.
        Vx(end+1,:)=Vx(end,:) + ux(end,:)*ScaleU*Tstep;
        Vy(end+1,:)=Vy(end,:) + uy(end,:)*ScaleU*Tstep;

%         count1=count1+1;
    end
end