% Obstacle function:
% Author: K. Passino, Version: 25/01/2001
% Modified: Arijit Kumar Haldar
function J=obstaclefunction(x,w1,obs)

% An function to represent sensed obstacles:
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