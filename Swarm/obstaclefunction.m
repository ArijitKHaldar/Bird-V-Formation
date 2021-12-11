% Obstacle function:
% Author: K. Passino, Version: 1/25/01
function J=obstaclefunction(x,w1)

% An example function to represent sensed obstacles:
    obs = [7,17;
        9,17;
        12,17;
        15,17;
        17,17;
        10,15;
        12,14;
        14,15;
        9,14;
        12,12;
        15,14;
        10,19;
        12,20;
        14,19;
        9,20;
        12,22;
        15,20];
    for i=1:size(obs,1)
        fn(1,i) = exp(-0.8*((x(1,1)-obs(i,1))^2 + (x(2,1)-obs(i,2))^2));
    end
    J = w1*max(fn);
% 	J=...
% 		w1*max([exp(-0.8*((x(1,1)-20)^2+(x(2,1)-15)^2)),...
% 		exp(-0.8*((x(1,1)-8)^2+(x(2,1)-10)^2)),...
% 		exp(-0.8*((x(1,1)-10)^2+(x(2,1)-10)^2)),...
% 		exp(-0.8*((x(1,1)-12)^2+(x(2,1)-10)^2)),...
% 		exp(-0.8*((x(1,1)-24)^2+(x(2,1)-20)^2)),...
% 		exp(-0.8*((x(1,1)-18)^2+(x(2,1)-20)^2))]);
end