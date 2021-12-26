% Main Code
% (Information and data dictionary pending)
%
% Please install "ffmpeg" and add to path before running this code for conversion to mp4 to work.


%% Initialization
clear all
close all hidden
clc

N=input('Enter number of agents (minimum 3): ');	% The number of agents (individuals) in swarm
flagg=input('Do you want to see animated plot? \n[For yes enter 1, for NO enter 0] : ');
if isempty(flagg)
    flagg = 0;
elseif flagg~=0 && flagg ~=1
    flagg = 0;
end
clc
tic % Starting timer to start calculating elapsed time
fprintf("\nStart simulation\n")

k1=1;	% Chosen to get a stability property ('kp')
k2=k1;  % Choose this k2>=0. Velocities converge to mean faster with larger k2. 
kv=0.1;   % Velocity damping ('k')  (kv=10 for quadratic case?)
kf=0.1;   % Gain on profile following (kf=50 for quadratic case?)
b=10;  % Define parameters for repel function ('kr')
c=1;  % Define parameter of repulsion region ('rs^2')

% Define simulation parameters:
Tfinal=6; % Units are seconds (keep it even)
Tstep=0.1; % Was 0.01
Tspan=0:Tstep:Tfinal+Tstep;

% Define initial conditions:
ICsize1=2; 
ICsize2=2;
X0=ICsize1*rand(1,N)+3;   % Pick random values for initial positions in X and Y dimensions
Y0=ICsize1*rand(1,N)+3;
Vx0=ICsize2*rand(1,N);    % Pick random values for initial velocities in X and Y dimensions
Vy0=ICsize2*rand(1,N);

% Initialization of position and velocity
X(1,1:N)=X0; Y(1,1:N)=Y0; % First dimension is time, second is N values of X (Y) position
Vx(1,1:N)=Vx0; Vy(1,1:N)=Vy0;
X_nth(1,1:N)=X0;
Y_nth(1,1:N)=Y0;
Vx_nth(1,1:N)=Vx0;
Vy_nth(1,1:N)=Vy0;

% Goal position of vehicle
xgoal=[25; 
       25];
w1=120; 		% Set weighting factors on the goal function and obstacle function
w2=0.1;

% Redefine the noise bound. 
value1=60;% The magnitude of noise for discrete time case.
Dp2=value1/10; 
Dv2=value1/10; 
Df=value1;

Dmax=1; % The magnitude of the noise. Since we use uniform noise of Matlab here, Dmax=1.
ScaleU=10; % This is used to change the magnitude of the control input ux and uy.
xrepel=zeros(1,N);
yrepel=zeros(1,N);

% Sensing parameters
k1_sense=2.5;	
k2_sense=2.5;   
kv_sense=0.35;   
kf_sense=0.35;   
b_sense=15;  
c_sense=1;  

%% Calculate values for each step

fprintf('\nSensing & Circle fitting...Please Wait...\nThis might take a long time...\n')

for n=1:Tfinal/Tstep-1

    %tic
    if(n==1)
        [X_virAgent,Y_virAgent,Vx_virAgent,Vy_virAgent]=mapNearbySpace_desTraj(k1_sense,k2_sense,kv_sense,kf_sense,b_sense,c_sense,xgoal);
    else 
        [X_virAgent,Y_virAgent,Vx_virAgent,Vy_virAgent]=mapNearbySpace_up_desTraj(k1_sense,k2_sense,kv_sense,kf_sense,b_sense,c_sense,X_virAgent,Y_virAgent,Vx_virAgent,Vy_virAgent,xgoal);
    end
    %fprintf('\nSensing\n')
    %toc

    X_VAgent(n+1,:) = X_virAgent(end,:);
    Y_VAgent(n+1,:) = Y_virAgent(end,:);
    
    % create more span
    coor_x = [X_virAgent(end-5,:);X_virAgent(end,:)];
    coor_y = [Y_virAgent(end-5,:);Y_virAgent(end,:)];
    
    % fit in a circular region around the sensing coordinates
    %tic
    P = CircleFitByPratt([coor_x(:)';coor_y(:)']);
    %fprintf('\n\nmainCircle fit\n')
    %toc
    
    loc_spline = [P(1,1) P(1,2)];
    cirCenter(n,:) = loc_spline; % This has the coordinates of center of circle after fitting is done
    
    [X_dash,Y_dash] = findCirclePoints(P);
    %X_dash = X_dash';
    %Y_dash = Y_dash';
    %Circle_Co{:,n} = [X_dash;Y_dash];
    Circle_Co = [X_dash,Y_dash]; % This has 50 coordinates on the circumference of the circle

    vertCoor = triangleVertices(n,P,cirCenter,Circle_Co);
   
    % Set coordinates of vertices of triangle for starting formation
    % This will contain not just the vertex coordinates, rather coordinates of all N agents where they need to be placed next
    pos_target=triangleAgents(N,vertCoor,c);
    
    [Xtemp,Ytemp,VxTemp,VyTemp,intrmdtSteps(1,n)] = updatePosition(X,Y,Vx,Vy,k1,k2,kv,N,b,c,Tstep,ScaleU,pos_target);
    X = Xtemp(end,:); % Overwriting previous X with new value (1 X N)
    Y = Ytemp(end,:); % Overwriting previous Y with new value (1 X N)
    Vx = VxTemp(end,:); % Overwriting previous Vx with new value (1 X N)
    Vy = VyTemp(end,:); % Overwriting previous Vy with new value (1 X N)
    
    X_nth(n+1,:) = Xtemp(end,:); % This is (n X N) X-values
    Y_nth(n+1,:) = Ytemp(end,:); % This is (n X N) Y-values
    Vx_nth(n+1,:) = VxTemp(end,:);
    Vy_nth(n+1,:) = VyTemp(end,:);
    
    X_all{:,n} = Xtemp; % {1 X n}(intrmdtSteps X N)
    Y_all{:,n} = Ytemp;
    Vx_all{:,n} = VxTemp;
    Vy_all{:,n} = VyTemp;
% Debugging code here    
%    plot(Circle_Co(:,1),Circle_Co(:,2));
%    hold on;
%    plot(X,Y,'m*','LineWidth',2);
%    axis([-5 40 -5 40]);
%    deleteThis = [7,17;
%        9,17;
%        12,17;
%        15,17;
%        17,17;
%        10,15;
%        12,14;
%        14,15;
%        9,14;
%        12,12;
%        15,14;
%        10,19;
%        12,20;
%        14,19;
%        9,20;
%        12,22;
%        15,20];
%    plot(deleteThis(:,1),deleteThis(:,2),'bs','LineWidth',3);
%    plot(xgoal(1),xgoal(2),'gx','MarkerSize',16,'linewidth',2);
%    hold off;
%    M(:,n)=getframe(gcf);
% Debugging code here    
end
toc

t=(1:length(X_nth))'*Tstep; 
var=0; % Just for convenience such that the plot commands below, which was for continous time case, are still valid.



%% Plotting

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot the swarm trajectories
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic
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

[temp1,temp2]=size(X_nth);
if (Tfinal>=20)
    temparray = 1:(Tfinal/20)*5:temp1;
else
    temparray = 1:temp1;
end

figure(1) %Plot initialized agents' positions and final goal coordinate
    clf
    contour(xx,yy,zz+zzz,40)
    colormap(jet)
    % Use next line for generating plots to put in black and white documents.
    %colormap(gray)
    hold on;
    xlabel('x')
    ylabel('y')
    title('J=w_1J_o + w_2J_g and initial (square) and goal (x) positions');
    % Plot initial and final positions
    plot(xgoal(1),xgoal(2),'gx','MarkerSize',16,'linewidth',2);
    plot(X0,Y0,'bs')
    hold off;


figure(2)
    clf
% Plot variation of position of agents along X and Y axes
    subplot(2,2,1)
    grid on;
    hold on;
    plot(t(temparray,:),X_nth(temparray,:),'linewidth',1)
    axis([0, floor(max(t)/10)*10*(t(end)>=10)+t(end)*(t(end)<10), floor(min(min(X_nth))/10)*10, ceil(max(max(X_nth))/10)*10]);
    title('Swarm agent position trajectories, x dimension')
    xlabel('Time, sec.')
    hold off;

    subplot(2,2,3)
    grid on;
    hold on;
    plot(t(temparray,:),Y_nth(temparray,:),'linewidth',1)
    axis([0, floor(max(t)/10)*10*(t(end)>=10)+t(end)*(t(end)<10), floor(min(min(Y_nth))/10)*10, ceil(max(max(Y_nth))/10)*10]);
    title('Swarm agent position trajectories, y dimension')
    xlabel('Time, sec.')
    hold off;

% Plot variation of velocity of agents along X and Y axes
    subplot(2,2,2)
    grid on;
    hold on;
    plot(t(temparray,:),Vx_nth(temparray,:),'linewidth',1)
    axis([0, floor(max(t)/10)*10*(t(end)>=10)+t(end)*(t(end)<10), floor(min(min(Vx_nth))/10)*10, ceil(max(max(Vx_nth))/10)*10]);
    title('Deviation from mean velocity, x dimension')
    xlabel('Time, sec.')
    axis([0,Tfinal,-20,20]);
    hold off;

    subplot(2,2,4)
    grid on;
    hold on;
    plot(t(temparray,:),Vy_nth(temparray,:),'linewidth',1)
    axis([0, floor(max(t)/10)*10*(t(end)>=10)+t(end)*(t(end)<10), floor(min(min(Vy_nth))/10)*10, ceil(max(max(Vy_nth))/10)*10]);
    title('Deviation from mean velocity, y dimension')
    xlabel('Time, sec.')
    axis([0,Tfinal,-20,20]);
    hold off;


figure(3) % Plot trajectory of path taken by agents to reach goal from beginning
    clf
    contour(xx,yy,zz+zzz,40)
    colormap(jet)
    hold on;
    plot(xgoal(1),xgoal(2),'gx','MarkerSize',16,'linewidth',2);
    title('Swarm agent position trajectories')
    plot(X_nth(temparray,:),Y_nth(temparray,:),'LineStyle',':') 
    xlabel('x')
    ylabel('y')
    plot(X0,Y0,'bs')
    plot(X_nth(temp1,:),Y_nth(temp1,:),'ro','LineWidth',2);
    hold off;
fprintf("\nEnd of plotting using %d seconds as simulation time.\n",Tfinal)
toc

% Next, produce a movie:
% Set flagg to 1 if want to see a movie
Xd=[];
Yd=[];
if flagg==1
    tic
    fprintf("\nStarting animated plot...please wait...\n")
    
    figure(4)
    clf
    axis([min(min(X_nth)) max(max(X_nth)) min(min(Y_nth)) max(max(Y_nth))]);
    
    R=20; % Set decimate factor
    
    for i=1:N
        
        Xd(:,i)=decimate(X(:,i),R);    		% Decimate data to speed up movie
        Yd(:,i)=decimate(Y(:,i),R);
        
    end
    
    [temp1d,temp2]=size(Xd);
    
    cd ..
    [success,message,messageid] = mkdir(pwd,'Outputs');
    cd Outputs
    if ispc
        ddtt=sprintf('%s\\test_%s.avi',pwd,datetime('now','TimeZone','Asia/Kolkata','Format','d-MMM-y_HH.mm'));
        cd ..\Swarm
    else
        ddtt=sprintf('%s/test_%s.avi',pwd,datetime('now','TimeZone','Asia/Kolkata','Format','d-MMM-y_HH.mm'));
        cd ../Swarm
    end
    videosave=VideoWriter(ddtt,'Uncompressed AVI');
    open(videosave);
    for j=1:temp1d
        clf;
        contour(xx,yy,zz+zzz,40)
        colormap(jet);
        hold on;
        plot(xgoal(1),xgoal(2),'gx','MarkerSize',16,'linewidth',2);
        plot(Xd(j,:),Yd(j,:),'ro','LineWidth',2);
        %axis([min(min(X)) max(max(X)) min(min(Y)) max(max(Y))]);
        axis([-2 30 -2 30]);
        xlabel('x')
        ylabel('y')
        title('Swarm agent position trajectories')
        M(:,j) = getframe(gcf);
        writeVideo(videosave,M(:,j));
    end
    close(videosave);
    
    hold on % Next, add as the last frame the set of trajectories and end/start points
    plot(X,Y,'k:');
    xlabel('x')
    ylabel('y')
    title('Swarm agent position trajectories')
    plot(X0,Y0,'bs');
    plot(X(temp1,:),Y(temp1,:),'ro','LineWidth',2);
    plot(xgoal(1),xgoal(2),'gx','MarkerSize',16,'linewidth',2);
    %M(:,temp1d+1)=getframe; % Add last frame as figure(1)
    
    % Play the movie
    %movie(M)
    fprintf("End of animated plot\n")
    toc
    
    % convert AVI to MP4
    fprintf("\nJust wait a few moments...converting to mp4\n\n")
    pathVideoMP4 = regexprep(ddtt,'\.avi','.mp4'); % generate mp4 filename
    if isunix % for linux
        [~,~] = system(sprintf('ffmpeg -i %s -y -an -c:v libx264 -crf 0 -preset slow %s',ddtt,pathVideoMP4)); % for this to work, you should have installed ffmpeg and have it available on PATH
    elseif ispc % for windows
        [~,~] = system(sprintf('ffmpeg.exe -i %s -y -an -c:v libx264 -crf 0 -preset slow %s',ddtt,pathVideoMP4)); % for this to work, you should have installed ffmpeg and have it available on PATH
    elseif ismac % for mac
        [~,~] = system(sprintf('ffmpeg -i %s -y -an -c:v libx264 -crf 0 -preset slow %s',ddtt,pathVideoMP4)); %  for this to work, you should have installed ffmpeg and have it available on PATH
    end
    fprintf("End of Simulation\n")
end