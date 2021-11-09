clear all
close all
clc
%load randstate; rand('state',rand_state);randn('state',randn_state);

tic

N=10;	% The number of agents (individuals) in swarm
k1=1;	% Chosen to get a stability property ('kp')
k2=k1;  % Choose this k2>=0. Velocities converge to mean faster with larger k2. 
kv=0.1;   % Velocity damping ('k')  (kv=10 for quadratic case?)
kf=0.1;   % Gain on profile following (kf=50 for quadratic case?)
b=10;  % Define parameters for repel function ('kr')
c=1;  % Define parameter of rulpulsion region ('rs^2') 

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
w2=0.1;

% Redefine the noise bound. 
value1=60;% The magnitude of noise for discrete time case.
Dp2=value1/10; 
Dv2=value1/10; 
Df=value1;

Dmax=1; % The magnitude of the noise. Since we use uniform noise of Matlab here, Dmax=1.
ScaleU=10; % This is used to change the magnitude of the control input ux and uy. %10

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

    EP_hat=[X(n,:); Y(n,:)]; 
    % 2xN, [EP_hat(1,i); EP_hat(2,i)] is the position error of agent i with sensing error.
    % Note above that in 2d case, only the first two rows of 'dp' (which is 3xN) are used.
    
    % The 'for' loop below caculates the effect of the repel term on each agent (in 3 dimensions).
    for i=1:N
        Ediff=EP_hat(:,i)*ones(1,N)-EP_hat; % 2xN matrix. Column j (1<=j<=N) contains the error position difference of agent i and agent j in [x;y] direction, respectively.
        dist=sqrt(sum(Ediff.*Ediff)); % 1xN vector. The jth component is the norm of the error difference of agent i and j. It's equal to the distance from agent i to agent j; or .
        xrepel(i)=sum(b*exp(-dist.^2/c).*(X(n,i)-X(n,:)));
        yrepel(i)=sum(b*exp(-dist.^2/c).*(Y(n,i)-Y(n,:)));
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
    ux=-k1*(X(n,:)-mean(X(n,:))) - k2*(Vx(n,:)-mean(Vx(n,:))) - kv*Vx(n,:) + xrepel - kf*(A(:,1)'); %Note A
    uy=-k1*(Y(n,:)-mean(Y(n,:))) - k2*(Vy(n,:)-mean(Vy(n,:))) - kv*Vy(n,:) + yrepel - kf*(A(:,2)');
    
    % Calculates the position and velocity in the next time step (Euler's method).
    X(n+1,:)=X(n,:)+Vx(n,:)*Tstep;
    Y(n+1,:)=Y(n,:)+Vy(n,:)*Tstep;
    Vx(n+1,:)=Vx(n,:) + ux*ScaleU*Tstep;
    Vy(n+1,:)=Vy(n,:) + uy*ScaleU*Tstep;
    
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
%colormap(gray)
hold on
plot(xgoal(1),xgoal(2),'gx','MarkerSize',16,'linewidth',2);
hold on
plot(X(temparray,:),Y(temparray,:),'LineStyle',':') 
xlabel('x')
ylabel('y')
title('Swarm agent position trajectories')
hold on
plot(X0,Y0,'bx')
hold on
plot(X(temp1,:),Y(temp1,:),'ro');
hold off
toc

% Next, produce a movie:
%flagg=1;  % Set to 0 if want to see a movie
flagg=0;
Xd=[];Yd=[];
tic
if flagg~=1
    
    figure(2)
    clf
    axis([min(min(X)) max(max(X)) min(min(Y)) max(max(Y))]);
    
    R=20; % Set decimate factor
    
    for i=1:N
        
        Xd(:,i)=decimate(X(:,i),R);    		% Decimate data to speed up movie
        Yd(:,i)=decimate(Y(:,i),R);
        
    end
    
    [temp1d,temp2]=size(Xd);
    
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
        title('Swarm agent position trajectories')
        M(:,j) = getframe;
        
    end
    
    hold on % Next, add as the last frame the set of trajectories and end/start points
    plot(X,Y,'k:');
    xlabel('x')
    ylabel('y')
    title('Swarm agent position trajectories')
    hold on
    plot(X0,Y0,'bx');
    hold on
    plot(X(temp1,:),Y(temp1,:),'ro');
    hold on;
    plot(xgoal(1),xgoal(2),'gx','MarkerSize',16,'linewidth',2);
    
    %M(:,temp1d+1)=getframe; % Add last frame as figure(1)
    
    % Play the movie
    
    %movie(M)
end
toc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of program
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
