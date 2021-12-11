function coordinate = trianglePeak(n,P,cirCenter,Circle_Co)
    if(n == 1) % For first iteration, align using global origin
       x1 = P(1,1);
       x2 = 0;
       y1 = P(1,2);
       y2 = 0;
       m = (y2-y1)/(x2-x1);
       r = P(1,3)-3;
       yintercept = 0;
       [xtemp,ytemp] = linecirc(m,yintercept,x1,y1,r);
       if xtemp(1,1) > xtemp(1,2)
           minimum =1000;
           for i=1:1:length(Circle_Co)
               if(sqrt(power(Circle_Co(i,1)-xtemp(1,1),2)+power(Circle_Co(i,2)-ytemp(1,1),2)) < minimum)
                   minimum = sqrt(power(Circle_Co(i,1)-xtemp(1,1),2)+power(Circle_Co(i,2)-ytemp(1,1),2));
                   Coor(1,:) = [Circle_Co(i,1),Circle_Co(i,2)];
               end
           end         
       else
           minimum =1000;
           for i=1:1:length(Circle_Co)
               if(sqrt(power(Circle_Co(i,1)-xtemp(1,2),2)+power(Circle_Co(i,2)-ytemp(1,2),2)) < minimum)
                   minimum = sqrt(power(Circle_Co(i,2)-xtemp(1,2),2)+power(Circle_Co(i,2)-ytemp(1,2),2));
                   Coor(1,:) = [Circle_Co(i,1),Circle_Co(i,2)];
               end
           end
       end
       
       
    else % For later, align using previous circle centre
        x1 = P(1,1);
        x2 = cirCenter(n-1,1);
        y1 = P(1,2);
        y2 = cirCenter(n-1,2);
        m = (y2-y1)/(x2-x1);
        r = P(1,3)-3;
        yintercept = (y1*x2-y2*x1)/(x2-x1);
        [xtemp,ytemp] = linecirc(m,yintercept,x1,y1,r);
        if xtemp(1,1) > xtemp(1,2)
           minimum =1000;
           for i=1:1:length(Circle_Co)
               if(sqrt(power(Circle_Co(i,1)-xtemp(1,1),2)+power(Circle_Co(i,2)-ytemp(1,1),2)) < minimum)
                   minimum = sqrt(power(Circle_Co(i,1)-xtemp(1,1),2)+power(Circle_Co(i,2)-ytemp(1,1),2));
                   Coor(1,:) = [Circle_Co(i,1),Circle_Co(i,2)];
               end
           end
       else
           minimum =1000;
           for i=1:1:length(Circle_Co)
               if(sqrt(power(Circle_Co(i,1)-xtemp(1,2),2)+power(Circle_Co(i,2)-ytemp(1,2),2)) < minimum)
                   minimum = sqrt(power(Circle_Co(i,2)-xtemp(1,2),2)+power(Circle_Co(i,2)-ytemp(1,2),2));
                   Coor(1,:) = [Circle_Co(i,1),Circle_Co(i,2)];
               end
           end
        end
    end
    coordinate = Coor(1,:);
end