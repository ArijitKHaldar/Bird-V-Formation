%% Initialization
clear
close all hidden
clc

N=input('Enter number of agents[Please enter 5 for now] :');	% The number of agents (individuals) in swarm
flagg=input('Do you want to see animated plot? (yes/NO -> 1/0)[Press ENTER for NO]: ');
if isempty(flagg)
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
c=1;  % Define parameter of rulpulsion region ('rs^2')

% Define simulation parameters:
Tfinal=80; % Units are seconds (keep it even)
Tstep=0.01;
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
    
    % plot the circular region over sensing info
%     figure;
%     scatter(coor_x(:),coor_y(:))
%     hold on
%     scatter(X_dash,Y_dash)

    [xtemp,ytemp] = linecirc(((0-cirCenter(1,2))/(0-cirCenter(1,1))),0,cirCenter(n,1),cirCenter(n,2),sqrt(power(Circle_Co(1,1)-cirCenter(n,1),2)+power(Circle_Co(1,2)-cirCenter(n,2),2)));
    if xtemp(1,1) > xtemp(1,2)
        minimum =1000;
        for i=1:1:length(Circle_Co)
            if(sqrt(power(Circle_Co(i,1)-xtemp(1,1),2)+power(Circle_Co(i,2)-xtemp(1,1),2)) < minimum)
                minimum = sqrt(power(Circle_Co(i,1)-xtemp(1,1),2)+power(Circle_Co(i,2)-xtemp(1,1),2));
                vertCoor(1,:) = [Circle_Co(i,1),Circle_Co(i,2)];
            end
        end
    else
        minimum =1000;
        for i=1:1:length(Circle_Co)
            if(sqrt(power(Circle_Co(i,1)-xtemp(1,2),2)+power(Circle_Co(i,2)-xtemp(1,2),2)) < minimum)
                minimum = sqrt(power(Circle_Co(i,2)-xtemp(1,2),2)+power(Circle_Co(i,2)-xtemp(1,2),2));
                vertCoor(1,:) = [Circle_Co(i,1),Circle_Co(i,2)];
            end
        end
    end
  
    tmp = 1;
    for i=1:1:length(Circle_Co)
        s = sqrt(power((Circle_Co(i,1)-cirCenter(n,1)),2)+power((Circle_Co(i,2)-cirCenter(n,2)),2))*sqrt(3);
        dist = sqrt(power((vertCoor(1,1)-Circle_Co(i,1)),2)+power((vertCoor(1,2)-Circle_Co(i,2)),2));
        if(abs(dist-s) < 0.16)
            vertCoor(tmp+1,:) = [Circle_Co(i,1),Circle_Co(i,2)];
            tmp=tmp+1;
        end
    end
   
    % Set coordinates of vertices of triangle for starting formation
    % Here I plan to add a function that returns Nx2 array for agent starting formation
    tria_form=trianglecoordinates(N,vertCoor);
    if (n==1)
        pos_targetNew = tria_form;
    end
    
    % Save the position and velocity of each agent at current n.
    pos_begin=[X(n,:)' Y(n,:)']; % Forms a N X 2 array
    vbar=mean([Vx(n,:)' Vy(n,:)']);
    
    % ErrorMatrix: 4xN, each column represents the error terms ([ep_x;ep_y;ev_x;ev_y]) of an agent.
    ErrorMatrix=[X(n,:)'-pos_targetNew(:,1) Y(n,:)'-pos_targetNew(:,2) Vx(n,:)'-vbar(:,1) Vy(n,:)'-vbar(:,2)]'; % Not used anywhere !!

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
    A=zeros(N,2);
    for i=1:N
        NowJ=goalfunction0([X(n,i);Y(n,i)],xgoal,w2) + obstaclefunction([X(n,i);Y(n,i)],w1);
        partial_x=Vx(n,i)*Tstep;
        partial_y=Vy(n,i)*Tstep;
        partialJx=goalfunction0([X(n,i)+partial_x;Y(n,i)],xgoal,w2) + obstaclefunction([X(n,i)+partial_x;Y(n,i)],w1) - NowJ;
        partialJy=goalfunction0([X(n,i);Y(n,i)+partial_y],xgoal,w2) + obstaclefunction([X(n,i);Y(n,i)+partial_y],w1) - NowJ;        
        A(i,:)=[partialJx/partial_x partialJy/partial_y];
    end
    
    % Calculate the control input on two dimension x,y. Each u (i.e., ux, uy) is a 1xN vector.
    ux=-k1*(X(n,:)-pos_targetNew(:,1)') - k2*(Vx(n,:)-mean(Vx(n,:))) - kv*Vx(n,:) + xrepel - kf*(A(:,1)'); %Note A
    uy=-k1*(Y(n,:)-pos_targetNew(:,2)') - k2*(Vy(n,:)-mean(Vy(n,:))) - kv*Vy(n,:) + yrepel - kf*(A(:,2)');
    
    % Calculates the position and velocity in the next time step (Euler's method).
    X(n+1,:)=X(n,:)+Vx(n,:)*Tstep;
    Y(n+1,:)=Y(n,:)+Vy(n,:)*Tstep;
    
%     theta=deg2rad(0); % I need to somehow find angle between position now and required position and put here
%     R = [cos(theta) -sin(theta); 
%         sin(theta) cos(theta)];
%     pos_targetOld = zeros(N,2);
%     while(rad2deg(theta) > 5.0 || rad2deg(theta) < -5.0)
%         pos_targetOld = (R*(pos_targetNew'-pos_targetNew(1,:)')+pos_targetNew(1,:)')';
%         pos_targetOld(:,1)=pos_targetOld(:,1)+0.4*Tstep;
%         pos_targetOld(:,2)=pos_targetOld(:,2)+0.4*Tstep;
%         theta = theta - deg2rad(2); % Sense the present angle and update theta to know how much more rotation needed
%         
%         pos_targetNew = pos_targetOld;
%         
%     end
    
    if sqrt(power(xgoal(1,1)-pos_targetNew(1,1),2)+power(xgoal(2,1)-pos_targetNew(1,2),2)) < 0.5
        pos_targetNew(:,1)=pos_targetNew(:,1)+0*Tstep;
        pos_targetNew(:,2)=pos_targetNew(:,2)+0*Tstep;
    else
        pos_targetNew(:,1)=pos_targetNew(:,1)+(sqrt(power(xgoal(1,1)-pos_targetNew(1,1),2)+power(xgoal(2,1)-pos_targetNew(1,2),2))/sqrt(power(xgoal(1,1)-X0(1,1),2)+power(xgoal(2,1)-Y0(1,1),2)))*Tstep;
        pos_targetNew(:,2)=pos_targetNew(:,2)+(sqrt(power(xgoal(1,1)-pos_targetNew(1,1),2)+power(xgoal(2,1)-pos_targetNew(1,2),2))/sqrt(power(xgoal(1,1)-X0(1,1),2)+power(xgoal(2,1)-Y0(1,1),2)))*Tstep;
    end
    Vx(n+1,:)=Vx(n,:) + ux*ScaleU*Tstep;
    Vy(n+1,:)=Vy(n,:) + uy*ScaleU*Tstep;
    
end
toc

t=(1:length(X))'*Tstep; 
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

[temp1,temp2]=size(X);
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
    plot(t(temparray,:),X(temparray,:),'linewidth',1)
    axis([0, floor(max(t)/10)*10*(t(end)>=10)+t(end)*(t(end)<10), floor(min(min(X))/10)*10, ceil(max(max(X))/10)*10]);
    title('Swarm agent position trajectories, x dimension')
    xlabel('Time, sec.')
    hold off;

    subplot(2,2,3)
    grid on;
    hold on;
    plot(t(temparray,:),Y(temparray,:),'linewidth',1)
    axis([0, floor(max(t)/10)*10*(t(end)>=10)+t(end)*(t(end)<10), floor(min(min(Y))/10)*10, ceil(max(max(Y))/10)*10]);
    title('Swarm agent position trajectories, y dimension')
    xlabel('Time, sec.')
    hold off;

% Plot variation of velocity of agents along X and Y axes
    subplot(2,2,2)
    grid on;
    hold on;
    plot(t(temparray,:),Vx(temparray,:),'linewidth',1)
    axis([0, floor(max(t)/10)*10*(t(end)>=10)+t(end)*(t(end)<10), floor(min(min(Vx))/10)*10, ceil(max(max(Vx))/10)*10]);
    title('Swarm velocities, x dimension')
    xlabel('Time, sec.')
    axis([0,Tfinal,-20,20]);
    hold off;

    subplot(2,2,4)
    grid on;
    hold on;
    plot(t(temparray,:),Vy(temparray,:),'linewidth',1)
    axis([0, floor(max(t)/10)*10*(t(end)>=10)+t(end)*(t(end)<10), floor(min(min(Vy))/10)*10, ceil(max(max(Vy))/10)*10]);
    title('Swarm velocities, y dimension')
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
    plot(X(temparray,:),Y(temparray,:),'LineStyle',':') 
    xlabel('x')
    ylabel('y')
    plot(X0,Y0,'bs')
    plot(X(temp1,:),Y(temp1,:),'ro','LineWidth',2);
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
    axis([min(min(X)) max(max(X)) min(min(Y)) max(max(Y))]);
    
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