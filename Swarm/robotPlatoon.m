% Forked from Kevin M. Passino's 
% "Cooperative Robot Swarm Obstacle Avoidance MATLAB simulation"
% [URL](http://eewww.eng.ohio-state.edu/~passino/ICbook/ic_code.html)
%
% Master of Control System Engineering Dissertation work
%
% * Author: Arijit Kumar Haldar
% * Guide: Prof. Madhubanti Maitra, Mr. Dibyendu Roy
% * Department: Electrical Engineering
% * University: Jadavpur University, West Bengal, India.
% Nov 3, 2021 - Present
%
% |   Variable   |   Order   |  Value   |           Description           |
% |--------------|-----------|----------|---------------------------------|
% |b             |1x1        |10        |Repulsion gain - Kr              |
% |b_sense       |1x1        |15        |Sensing Parameter - Details?     |
% |c             |           |          |                                 |
% |c_sense       |           |          |                                 |
% |cirCenter     |           |          |                                 |
% |Circle_Co     |           |          |                                 |
% |coor_x        |           |          |                                 |
% |coor_y        |           |          |                                 |
% |Df            |           |          |                                 |
% |Dmax          |           |          |                                 |
% |Dp2           |           |          |                                 |
% |Dv2           |           |          |                                 |
% |flag          |           |          |                                 |
% |ICsize1       |           |          |                                 |
% |ICsize2       |           |          |                                 |
% |intrmdtSteps  |           |          |Total no. of sub-steps for each n|
% |k1            |           |          |                                 |
% |k1_sense      |           |          |                                 |
% |k2            |           |          |                                 |
% |k2_sense      |           |          |                                 |
% |kf            |           |          |                                 |
% |kf_sense      |           |          |                                 |
% |kv            |           |          |                                 |
% |kv_sense      |           |          |                                 |
% |loc_spline    |           |          |                                 |
% |n             |           |          |                                 |
% |N             |           |          |                                 |
% |P             |           |          |                                 |
% |pos_target    |           |          |                                 |
% |ScaleU        |           |          |                                 |
% |t             |           |          |                                 |
% |Tfinal        |           |          |                                 |
% |Tspan         |           |          |                                 |
% |Tstep         |           |          |                                 |
% |value1        |           |          |                                 |
% |var           |           |          |                                 |
% |vertCoor      |           |          |                                 |
% |Vx            |           |          |                                 |
% |Vx0           |           |          |                                 |
% |Vx_all        |           |          |                                 |
% |Vx_nth        |           |          |                                 |
% |Vx_virAgent   |           |          |                                 |
% |VxTemp        |           |          |                                 |
% |Vy            |           |          |                                 |
% |Vy0           |           |          |                                 |
% |Vy_all        |           |          |                                 |
% |Vy_nth        |           |          |                                 |
% |Vy_virAgent   |           |          |                                 |
% |VyTemp        |           |          |                                 |
% |w1            |           |          |                                 |
% |w2            |           |          |                                 |
% |X             |           |          |                                 |
% |X0            |           |          |                                 |
% |X_all         |           |          |                                 |
% |X_dash        |           |          |                                 |
% |X_nth         |           |          |                                 |
% |X_VAgent      |           |          |                                 |
% |X_virAgent    |           |          |                                 |
% |xgoal         |           |          |                                 |
% |xrepel        |           |          |                                 |
% |Xtemp         |           |          |                                 |
% |Y             |           |          |                                 |
% |Y0            |           |          |                                 |
% |Y_all         |           |          |                                 |
% |Y_dash        |           |          |                                 |
% |Y_nth         |           |          |                                 |
% |Y_VAgent      |           |          |                                 |
% |Y_virAgent    |           |          |                                 |
% |yrepel        |           |          |                                 |
% |Ytemp         |           |          |                                 |
%
%
% Please install "ffmpeg" and add to path before running this code for conversion to mp4 to work.


%% Initialization
clear all
close all hidden
clc

N=input('Enter number of agents (minimum 3): ');	% The number of agents (individuals) in swarm
flag=input('Do you want to see animated plot? \n[For yes enter 1, for NO enter 0] : ');
if isempty(flag)
    flag = 0;
elseif flag~=0 && flag ~=1
    flag = 0;
end
vertCoor = zeros(3,2);
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
Tfinal=2; % Units are seconds (preferably even)
Tstep=0.01;
Tspan=0:Tstep:Tfinal+Tstep;

% Define initial conditions:
ICsize1=2; 
ICsize2=2; % Change this to make initialization more random
X0=ICsize1*rand(1,N)+3;   % Pick random values for initial positions in X and Y dimensions
Y0=ICsize1*rand(1,N)+3;
Vx0=ICsize2*rand(1,N);    % Pick random values for initial velocities in X and Y dimensions
Vy0=ICsize2*rand(1,N);

% Initialization of position and velocity
X(1,1:N)=X0; % First dimension is time, second is N values of X (Y) position
Y(1,1:N)=Y0; 

Vx(1,1:N)=Vx0; 
Vy(1,1:N)=Vy0;

X_nth(1,1:N)=X0;
Y_nth(1,1:N)=Y0;

Vx_nth(1,1:N)=Vx0;
Vy_nth(1,1:N)=Vy0;

X_all{:,1} = X0;
Y_all{:,1} = Y0;

Vx_all{:,1} = Vx0;
Vy_all{:,1} = Vy0;

intrmdtSteps(1,1)=1;

% Obstacle positions
count = 1;
for o_i=2:0.3:10
   obstacle(count,1) = o_i;
   obstacle(count,2) = o_i+15;
   count = count+1;
end
for o_i=10:0.3:35
   obstacle(count,1) = o_i;
   obstacle(count,2) = 25;
   count = count+1;
end
for o_i=35:0.3:43
   obstacle(count,1) = o_i;
   obstacle(count,2) = o_i-10;
   count = count+1;
end

% Goal position of vehicle
xgoal=[50; 
       50];
w1=120; 		% Set weighting factors on the goal function and obstacle function
w2=0.1;

% Redefine the noise bound. 
% value1=60;% The magnitude of noise for discrete time case.
% Dp2=value1/10; 
% Dv2=value1/10; 
% Df=value1;

% Dmax=1; % The magnitude of the noise. Since we use uniform noise of Matlab here, Dmax=1.
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
        [X_virAgent,Y_virAgent,Vx_virAgent,Vy_virAgent]=mapNearbySpace_desTraj(k1_sense,k2_sense,kv_sense,kf_sense,b_sense,c_sense,xgoal,obstacle);
    else 
        [X_virAgent,Y_virAgent,Vx_virAgent,Vy_virAgent]=mapNearbySpace_up_desTraj(k1_sense,k2_sense,kv_sense,kf_sense,b_sense,c_sense,X_virAgent,Y_virAgent,Vx_virAgent,Vy_virAgent,xgoal,obstacle);
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
    [P,Circle_Co,cirCenter_fn] = checkObstacleFree_CirclePlacement(obstacle,P);
    cirCenter(n,:) = cirCenter_fn;
    %X_dash = X_dash';
    %Y_dash = Y_dash';
    %Circle_Co{:,n} = [X_dash;Y_dash];
    vertCoor = triangleVertices(n,P,cirCenter,Circle_Co,xgoal,vertCoor);
   
    % Set coordinates of vertices of triangle for starting formation
    % This will contain not just the vertex coordinates, rather coordinates of all N agents where they need to be placed next
    pos_target=triangleAgents(N,vertCoor,c);
    
    [Xtemp,Ytemp,VxTemp,VyTemp,intrmdtSteps(1,n+1)] = updatePosition(X,Y,Vx,Vy,k1,k2,kv,N,b,c,Tstep,ScaleU,pos_target);
    X = Xtemp(end,:); % Overwriting previous X with new value (1 X N)
    Y = Ytemp(end,:); % Overwriting previous Y with new value (1 X N)
    Vx = VxTemp(end,:); % Overwriting previous Vx with new value (1 X N)
    Vy = VyTemp(end,:); % Overwriting previous Vy with new value (1 X N)
    
    X_nth(n+1,:) = Xtemp(end,:); % This is (n X N) X-values
    Y_nth(n+1,:) = Ytemp(end,:); % This is (n X N) Y-values
    Vx_nth(n+1,:) = VxTemp(end,:);
    Vy_nth(n+1,:) = VyTemp(end,:);
    
    X_all{:,n+1} = Xtemp; % {1 X n}(intrmdtSteps X N)
    Y_all{:,n+1} = Ytemp;
    Vx_all{:,n+1} = VxTemp;
    Vy_all{:,n+1} = VyTemp;
% Debugging code below for quick visualization
%    plot(Circle_Co(:,1),Circle_Co(:,2));
%    hold on;
%    plot(X,Y,'m*','LineWidth',2);
%    axis([-5 xgoal(1,1)+15 -5 xgoal(1,1)+15]);
%    plot(obstacle(:,1),obstacle(:,2),'b*');
%    plot(xgoal(1),xgoal(2),'gx','MarkerSize',16,'linewidth',2);
%    hold off;
%    M(:,n)=getframe(gcf);
% Debugging code above for quick visualization   
end
toc

t=(1:length(X_nth))'*Tstep; 
var=0; % Just for convenience such that the plot commands below, which was for continous time case, are still valid.

X_full=zeros(sum(intrmdtSteps),N);
Y_full=zeros(sum(intrmdtSteps),N);
Vx_full=zeros(sum(intrmdtSteps),N);
Vy_full=zeros(sum(intrmdtSteps),N);

counter1=0;
for pos_x_i=1:size(X_all,2)
    for pos_x_j=1:intrmdtSteps(1,pos_x_i)
        counter1=counter1+1;
        X_full(counter1,:)=X_all{1,pos_x_i}(pos_x_j,:);
    end
end
counter1=0;
for pos_y_i=1:size(Y_all,2)
    for pos_y_j=1:intrmdtSteps(1,pos_y_i)
        counter1=counter1+1;
        Y_full(counter1,:)=Y_all{1,pos_y_i}(pos_y_j,:);
    end
end
counter1=0;
for vel_x_i=1:size(Vx_all,2)
    for vel_x_j=1:intrmdtSteps(1,vel_x_i)
        counter1=counter1+1;
        Vx_full(counter1,:)=Vx_all{1,vel_x_i}(vel_x_j,:);
    end
end
counter1=0;
for vel_y_i=1:size(Vy_all,2)
    for vel_y_j=1:intrmdtSteps(1,vel_y_i)
        counter1=counter1+1;
        Vy_full(counter1,:)=Vy_all{1,vel_y_i}(vel_y_j,:);
    end
end
clear counter1;clear pos_x_i;clear pos_x_j;clear pos_y_i;clear pos_y_j;clear vel_x_i;clear vel_x_j;clear vel_y_i;clear vel_y_j;

%% Plotting
%load("nine_agents_no_plot.mat");
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot the swarm trajectories
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic
% Plot the functions:
kk=xgoal(1,1)+15;
xx=-5:(kk+1)/100:kk;   % For our function the range of values we are considering
yy=xx;

% Compute the obstacle and goal functions

for jj=1:length(xx)
	for ii=1:length(yy)
		zz(ii,jj)=obstaclefunction([xx(jj);yy(ii)],w1,obstacle);
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
    contour(xx,yy,zz+zzz,xgoal(1,1)+15)
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
    contour(xx,yy,zz+zzz,xgoal(1,1)+15)
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
% Set flag to 1 if want to see a movie
if flag==1
    Xd=[];
    Yd=[];
    tic
    fprintf("\nStarting animated plot...please wait...\n")
    
    figure(4)
    clf
%     axis([min(min(X_full)) max(max(X_full)) min(min(Y_full)) max(max(Y_full))]);
    
%     R=20; % Set decimate factor, i.e., downsample to 1/R the original sample rate
    
%     for i=1:N
%         
%         Xd(:,i)=decimate(X_full(:,i),R); % Decimate data to speed up movie, 8th order Chebyshev Type I LP filter with f_cutoff=0.8*(Fs/2)/R before resampling
%         Yd(:,i)=decimate(Y_full(:,i),R);
%         
%     end
    
%     [temp1d,temp2]=size(Xd);
    
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
    for j=1:size(X_full,1)
        clf;
        contour(xx,yy,zz+zzz,xgoal(1,1)+15)
        colormap(jet);
        hold on;
        plot(xgoal(1),xgoal(2),'gx','MarkerSize',16,'linewidth',2);
        plot(X_full(j,:),Y_full(j,:),'ro','LineWidth',2);
        axis([min(min(X_nth))-10 max(max(X_nth))+15 min(min(Y_nth))-10 max(max(Y_nth))+15]);
        xlabel('x')
        ylabel('y')
        title('Swarm agent position trajectories')
        M(:,j) = getframe(gcf);
        writeVideo(videosave,M(:,j));
    end
    close(videosave);
    
    hold on % Next, add as the last frame the set of trajectories and end/start points
    plot(X_full,Y_full,'k:');
    xlabel('x')
    ylabel('y')
    title('Swarm agent position trajectories')
    plot(X0,Y0,'bs');
    plot(X_nth(temp1,:),Y_nth(temp1,:),'ro','LineWidth',2);
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