%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robot swarm simulation in discrete time. Builds on
% simulation of robot path planning problem for 
% obstacle avoidance problem.
%
% Author: Yanfei Liu and Kevin Passino
% Version: April 24, 2003
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc
%randn('state',931316787);
%rand('state',787613139);
load randstate; rand('state',rand_state);randn('state',randn_state);
%randn('state',0);rand('state',0);
tic

N=30;	% The number of agents (individuals) in swarm
k1=1;	% Chosen to get a stability property ('kp')
k2=k1;  % Choose this k2>=0. Velocities converge to mean faster with larger k2. 
kv=0.1;   % Velocity damping ('k')  (kv=10 for quadratic case?)
kf=0.1;   % Gain on profile following (kf=50 for quadratic case?)
b=10;  % Define parameters for repel function ('kr')
c=1;  % Define parameter of rulpulsion region ('rs^2') 

NoiseOnOff=1; % This  parameter turn on or off the noise. '0' is 'off'.
LinearOnOff=1; % This parameter switch the noise bound between linear bound and constant bound. 1 is linear bound

% Define simulation parameters:
Tfinal=80; % Units are seconds (keep it even)
Tstep=0.01;
Tspan=0:Tstep:Tfinal+Tstep;

% Define initial conditions:
ICsize1=2; ICsize2=2;
X0=ICsize1*rand(1,N)+3;   % Pick random values for initial positions in X and Y dimensions
Y0=ICsize1*rand(1,N)+3;
Vx0=ICsize2*rand(1,N);    % Pick random values for initial velocities in X and Y dimensions
Vy0=ICsize2*rand(1,N);

% Initialization of position and velocity
X(1,1:N)=X0; Y(1,1:N)=Y0; % First dimension is time, second is N values of X (Y) position
Vx(1,1:N)=Vx0; Vy(1,1:N)=Vy0; 

% Goal position of vehicle
xgoal=[25; 25];
w1=120; 		% Set weighting factors on the goal function and obstacle function
				% w1=120 and w2=0.1 give good result ! 03/11/03
w2=1.0000e-01;

% Redefine the noise bound. 
value0=.5*LinearOnOff; value1=60;% The magnitude of noise for discrete time case.
Dp1=value0; Dp2=value1/10; Dv1=value0; Dv2=value1/10; Df=value1;

Dmax=1; % The magnitude of the noise. Since we use uniform noise of Matlab here, Dmax=1.
ScaleU=1; % This is used to change the magnitude of the control input ux and uy.

for n=1:Tfinal/Tstep-1
    
    % Note: We use uniform noise here. And a '2*' and '-1' is used to guarantee the noise 
    % is zero-mean, otherwise it won't be able to follow the given plane profile.
    dp1=2*rand(3,N)-1;
    dv1=2*rand(3,N)-1; 
    dp2=2*rand(3,N)-1; 
    dv2=2*rand(3,N)-1; 
    df0=2*rand(3,N)-1;
    
    % Calculate the average position and velocity of the swarm at current n.
    if (N>1) % Test to see if there is more than one swarm member
        xbar=mean([X(n,:)' Y(n,:)']);       % 2x1 vector of means in X and Y dimensions
        vbar=mean([Vx(n,:)' Vy(n,:)']);      % and for velocity also
    else
        xbar=[X(n,:)' Y(n,:)']; % If only one swarm member then mean is just that member's position
        vbar=[Vx(n,:)' Vy(n,:)'];
    end
    
    % ErrorMatrix: 4xN, each column represents the error terms ([ep_x;ep_y;ev_x;ev_y]) of an agent.
    ErrorMatrix=[X(n,:)-xbar(1); Y(n,:)-xbar(2); Vx(n,:)-vbar(1); Vy(n,:)-vbar(2)]; 
    %size(ErrorMatrix)
    
    normEi=LinearOnOff*sqrt(sum(ErrorMatrix.*ErrorMatrix)); %1xN vector, containing the error norm for each agent.
    dp=NoiseOnOff*(dp1/Dmax*Dp1.*(ones(3,1)*normEi) + dp2/Dmax*Dp2); % 3xN, [dp(1,i),dp(2,i),dp(3,i)] corresponds to dp[x,y,z] of agent i, respectively
    dv=NoiseOnOff*(dv1/Dmax*Dv1.*(ones(3,1)*normEi) + dv2/Dmax*Dv2); % 3xN, [dv(1,i),dv(2,i),dv(3,i)] corresponds to dv[x,y,z] of agent i, respectively
    df=NoiseOnOff*(df0/Dmax*Df); % 3xN, [df(1,i),df(2,i),df(3,i)] corresponds to df[x,y,z] of agent i, respectively
    
    %EP_hat=[X(n,:)-dp(1,:); Y(n,:)-dp(2,:); Z(n,:)-dp(3,:)]; % 3xN. For 3-d Case.
    EP_hat=[X(n,:)-dp(1,:); Y(n,:)-dp(2,:)]; % 2xN, [EP_hat(1,i); EP_hat(2,i)] is the position error of agent i with sensing error.
    % Note above that in 2d case, only the first two rows of 'dp' (which is 3xN) are used.
    
    % The 'for' loop below caculates the effect of the repel term on each agent (in 3 dimensions).
    for i=1:N
        Ediff=EP_hat(:,i)*ones(1,N)-EP_hat; % 2xN matrix. Column j (1<=j<=N) contains the error position difference of agent i and agent j in [x;y] direction, respectively.
        dist=sqrt(sum(Ediff.*Ediff)); % 1xN vector. The jth component is the norm of the error difference of agent i and j. It's equal to the distance from agent i to agent j; or .
        xrepel(i)=sum(b*exp(-dist.^2/c).*(X(n,i)-X(n,:)));
        yrepel(i)=sum(b*exp(-dist.^2/c).*(Y(n,i)-Y(n,:)));
        %zrepel(i)=sum(b*exp(-dist.^2/c).*(Z(n,i)-Z(n,:))); % For 3-d case.
    end
    % The 'for' loop below calculates the discrete gradient for each agent at current position.
    A=[];
    for i=1:N
        NowJ=goalfunction0([X(n,i);Y(n,i)],xgoal,w2) + obstaclefunction([X(n,i);Y(n,i)],w1);
        partial_x=Vx(n,i)*Tstep;
        partial_y=Vy(n,i)*Tstep;
        partialJx=goalfunction0([X(n,i)+partial_x;Y(n,i)],xgoal,w2) + obstaclefunction([X(n,i)+partial_x;Y(n,i)],w1) - NowJ;
        partialJy=goalfunction0([X(n,i);Y(n,i)+partial_y],xgoal,w2) + obstaclefunction([X(n,i);Y(n,i)+partial_y],w1) - NowJ;        
        A(i,:)=[partialJx/partial_x partialJy/partial_y];
    end
    
    % Calculate the control input on two dimension x,y. Each u (i.e., ux, uy) is a 1xN vector.
    ux=-k1*(X(n,:)-mean(X(n,:))-dp(1,:)) - k2*(Vx(n,:)-mean(Vx(n,:))-dv(1,:)) - kv*Vx(n,:) + xrepel - kf*(A(:,1)'-df(1,:));
    uy=-k1*(Y(n,:)-mean(Y(n,:))-dp(2,:)) - k2*(Vy(n,:)-mean(Vy(n,:))-dv(2,:)) - kv*Vy(n,:) + yrepel - kf*(A(:,2)'-df(2,:));
    %uz=-k1*(Z(n,:)-mean(Z(n,:))-dp(3,:)) - k2*(Vz(n,:)-mean(Vz(n,:))-dv(3,:)) - kv*Vz(n,:) + zrepel - kf*(A(:,3)'-df(3,:)); % For 3-d case.
    
    % Calculates the position and velocity in the next time step (Euler's method).
    X(n+1,:)=X(n,:)+Vx(n,:)*Tstep;
    Y(n+1,:)=Y(n,:)+Vy(n,:)*Tstep;
    %Z(n+1,:)=Z(n,:)+Vz(n,:)*Tstep; % For 3-d case.
    Vx(n+1,:)=Vx(n,:) + ux*ScaleU*Tstep;
    Vy(n+1,:)=Vy(n,:) + uy*ScaleU*Tstep;
    %Vz(n+1,:)=Vz(n,:) + uz*ScaleU*Tstep; % For 3-d case.
    
end

t=[1:length(X)]'*Tstep; var=0; % Just for convenience such that the plot commands below, which was for continous time case, are still valid.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot the swarm trajectories
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Plot the functions:
kk=40;
xx=-5:(kk+1)/100:kk;   % For our function the range of values we are considering
yy=xx;

% Compute the obstacle and goal functions

for jj=1:length(xx)
	for ii=1:length(yy)
		zz(ii,jj)=obstaclefunction([xx(jj);yy(ii)],w1);
	end
end
for jj=1:length(xx)
	for ii=1:length(yy)
		zzz(ii,jj)=goalfunction0([xx(jj);yy(ii)],xgoal,w2);
	end
end

[temp1,temp2]=size(X);
if (Tfinal>=20)
    temparray = 1:(Tfinal/20)*5:temp1;
else
    temparray = 1:temp1;
end

figure(1)
clf
contour(xx,yy,zz+zzz,40)
colormap(jet)
% Use next line for generating plots to put in black and white documents.
%colormap(gray);
xlabel('x');
ylabel('y');
title('J=w_1J_o + w_2J_g and initial (square) and goal (x) positions');
hold on

% Plot initial and final positions
%plot(5,5,'s'); hold on;
plot(xgoal(1),xgoal(2),'gx','MarkerSize',16,'linewidth',2);

plot(X(temparray,:),Y(temparray,:),'LineStyle',':') 
%plot3(X(temparray,:),Y(temparray,:),Z(temparray,:),'Color',[0.7,0.7,0.7],'LineStyle',':')
xlabel('x')
ylabel('y')
%zlabel('z') % For 3-d case.
title('Swarm agent position trajectories')
hold on
plot(X0,Y0,'bx')
%plot3(X0,Y0,Z0,'bx') % For 3-d case.
hold on
plot(X(temp1,:),Y(temp1,:),'ro');
%plot3(X(temp1,:),Y(temp1,:),Z(temp1,:),'ro') ;rotate3d on % For 3-d case.

figure(2) %(3) 
clf
subplot(211)
plot(t(temparray,:),X(temparray,:))
axis([0, floor(max(t)/10)*10*(t(end)>=10)+t(end)*(t(end)<10), floor(min(min(X))/10)*10, ceil(max(max(X))/10)*10]);
%plot(t(8000:end),X(8000:end,:),'b-')
title('Swarm agent position trajectories, x dimension')
xlabel('Time, sec.')
grid; hold on;
subplot(212)
plot(t(temparray,:),Y(temparray,:))
axis([0, floor(max(t)/10)*10*(t(end)>=10)+t(end)*(t(end)<10), floor(min(min(Y))/10)*10, ceil(max(max(Y))/10)*10]);
%plot(t(8000:end),Y(8000:end,:),'b-')
title('Swarm agent position trajectories, y dimension')
xlabel('Time, sec.')
grid; hold on;

figure(3) %(5) 
clf
subplot(211)
plot(t(temparray,:),Vx(temparray,:),'b-')
axis([0, floor(max(t)/10)*10*(t(end)>=10)+t(end)*(t(end)<10), floor(min(min(Vx))/10)*10, ceil(max(max(Vx))/10)*10]);
title('Swarm velocities, x dimension')
xlabel('Time, sec.')
axis([0,Tfinal,-20,20]); hold on;
grid
subplot(212)
plot(t(temparray,:),Vy(temparray,:),'b-')
axis([0, floor(max(t)/10)*10*(t(end)>=10)+t(end)*(t(end)<10), floor(min(min(Vy))/10)*10, ceil(max(max(Vy))/10)*10]);
title('Swarm velocities, y dimension')
xlabel('Time, sec.')
axis([0,Tfinal,-20,20]); hold on;
grid

toc
%break;

% Next, produce a movie:

%flagg=1;  % Set to 0 if want to see a movie
flagg=0;
Xd=[];Yd=[];
tic
if flagg~=1
    
    figure(5)
    clf
    axis([min(min(X)) max(max(X)) min(min(Y)) max(max(Y))]);
    
    R=20; % Set decimate factor
    
    for i=1:N
        
        Xd(:,i)=decimate(X(:,i),R);    		% Decimate data to speed up movie
        Yd(:,i)=decimate(Y(:,i),R);
        %Zd(:,i)=decimate(Z(:,i),R);
        
    end
    
    [temp1d,temp2]=size(Xd);
        
    %M = moviein(temp1d+1);
    
    for j=1:temp1d
        clf;
        contour(xx,yy,zz+zzz,40)
        colormap(jet);
        hold on;
        plot(xgoal(1),xgoal(2),'gx','MarkerSize',16,'linewidth',2);
        plot(Xd(j,:),Yd(j,:),'bo');
        axis([min(min(X)) max(max(X)) min(min(Y)) max(max(Y))]);
        xlabel('x')
        ylabel('y')
        %zlabel('z')
        title('Swarm agent position trajectories')
        M(:,j) = getframe;
        
    end
    
    hold on % Next, add as the last frame the set of trajectories and end/start points
    plot(X,Y,'k:');
    xlabel('x')
    ylabel('y')
    %zlabel('z')
    title('Swarm agent position trajectories')
    hold on
    plot(X0,Y0,'bx');
    hold on
    plot(X(temp1,:),Y(temp1,:),'ro');
    hold on;plot(xgoal(1),xgoal(2),'gx','MarkerSize',16,'linewidth',2);
    
    %M(:,temp1d+1)=getframe; % Add last frame as figure(1)
    
    % Play the movie
    
    %movie(M)
end
toc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of program
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%