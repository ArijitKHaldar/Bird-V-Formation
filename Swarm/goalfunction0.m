% Goal function: 
% Author: Y. Liu, Version: 03/11/03
function Jg=goalfunction0(x,xgoal,w2)
% Let m be the dimension of the space and n the number of "agents".
% Here xgoal is a mx1 vector indicates the position of the goal. 
% x is a mxn matrix, indicating the positions of n points in a m-dim space.
% And Jg returns a 1xn vector containing the cost function of those n points.
    [~,n]=size(x); 

    Err0= x-xgoal*ones(1,n);
    Jg=w2*sum(Err0.*Err0);
