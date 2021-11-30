function coordinate = trianglePeak(n,P,cirCenter,Circle_Co)
    if(n == 1)
       [xtemp,ytemp] = linecirc(((0-P(1,2))/(0-P(1,1))),0,P(1,1),P(1,2),P(1,3));
       if xtemp(1,1) > xtemp(1,2)
           minimum =1000;
           for i=1:1:length(Circle_Co)
               if(sqrt(power(Circle_Co(i,1)-xtemp(1,1),2)+power(Circle_Co(i,2)-xtemp(1,1),2)) < minimum)
                   minimum = sqrt(power(Circle_Co(i,1)-xtemp(1,1),2)+power(Circle_Co(i,2)-xtemp(1,1),2));
                   Coor(1,:) = [Circle_Co(i,1),Circle_Co(i,2)];
               end
           end
       else
           minimum =1000;
           for i=1:1:length(Circle_Co)
               if(sqrt(power(Circle_Co(i,1)-xtemp(1,2),2)+power(Circle_Co(i,2)-xtemp(1,2),2)) < minimum)
                   minimum = sqrt(power(Circle_Co(i,2)-xtemp(1,2),2)+power(Circle_Co(i,2)-xtemp(1,2),2));
                   Coor(1,:) = [Circle_Co(i,1),Circle_Co(i,2)];
               end
           end
       end
    else
        [xtemp,ytemp] = linecirc(((cirCenter(n-1,2)-P(1,2))/(cirCenter(n-1,1)-P(1,1))),((P(1,2)*cirCenter(n-1,1)-cirCenter(n-1,2)*P(1,1))/(cirCenter(n-1,1)-P(1,1))),P(1,1),P(1,2),P(1,3));
        if xtemp(1,1) > xtemp(1,2)
           minimum =1000;
           for i=1:1:length(Circle_Co)
               if(sqrt(power(Circle_Co(i,1)-xtemp(1,1),2)+power(Circle_Co(i,2)-xtemp(1,1),2)) < minimum)
                   minimum = sqrt(power(Circle_Co(i,1)-xtemp(1,1),2)+power(Circle_Co(i,2)-xtemp(1,1),2));
                   Coor(1,:) = [Circle_Co(i,1),Circle_Co(i,2)];
               end
           end
       else
           minimum =1000;
           for i=1:1:length(Circle_Co)
               if(sqrt(power(Circle_Co(i,1)-xtemp(1,2),2)+power(Circle_Co(i,2)-xtemp(1,2),2)) < minimum)
                   minimum = sqrt(power(Circle_Co(i,2)-xtemp(1,2),2)+power(Circle_Co(i,2)-xtemp(1,2),2));
                   Coor(1,:) = [Circle_Co(i,1),Circle_Co(i,2)];
               end
           end
       end
    end
    coordinate = Coor(1,:);
end