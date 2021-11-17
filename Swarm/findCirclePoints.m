function [x,y] = findCirclePoints(Par)

    th = linspace(0,2*pi,50)';
    R = Par(1,3)-3;
    x = R*cos(th)+Par(1,1);
    y = R*sin(th)+Par(1,2);
  
%    plot(x,y), title(' measured points')
end