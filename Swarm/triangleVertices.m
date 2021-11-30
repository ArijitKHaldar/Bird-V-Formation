function vertices = triangleVertices(n,P,cirCenter,Circle_Co)
    
    vertCoor(1,:) = trianglePeak(n,P,cirCenter,Circle_Co);
    
    tmp = 1;
    
    for i=1:1:length(Circle_Co)
    
        s = sqrt(power((Circle_Co(i,1)-cirCenter(n,1)),2)+power((Circle_Co(i,2)-cirCenter(n,2)),2))*sqrt(3); % r*sqrt(3)
        dist = sqrt(power((vertCoor(1,1)-Circle_Co(i,1)),2)+power((vertCoor(1,2)-Circle_Co(i,2)),2));
        
        if(abs(dist-s) < 0.16)
            vertCoor(tmp+1,:) = [Circle_Co(i,1),Circle_Co(i,2)];
            tmp=tmp+1;
        end
    end
    
    vertices = vertCoor;

end