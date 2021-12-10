% If number of agents is even, distribution will be assymetric
%
%               A
%
%          P1L     P1R
%
%      P2L             P2R
%
%    B                      C
%
function ft=triangleAgents(num,triangle_vertex,minDist)
     
     arr=zeros(num,2); % Preallocating
     
     arr(1,:)=triangle_vertex(1,:); % Coordinate of A
     arr(2,:)=triangle_vertex(2,:); % Coordinate of B
     arr(3,:)=triangle_vertex(3,:); % Coordinate of C
     
     if num > 3
         dist = sqrt(power((arr(1,1)-arr(2,1)),2)+power((arr(1,2)-arr(2,2)),2)); % Distance between points AB = AC
         numPoints = floor((num-2)/2); % Returns total number of agents that has to be fit between A and B
         subDist = dist/(numPoints+1); % Distance between B to P2L
         if subDist < minDist*2
                error('Not enough space to initialise %d agents',num);
         end
         % Now, applying the formula (P will have order of numPoints,2)
         % P = (m_1*x_2+m_2*x_1)/(m_1+m_2) . P_L will contain coordinates of agents for left arm of triangle, P_R will contain coordinate of right arm
         
         count = num-3;
         for i = 1:numPoints
             m_1 = i*subDist;
             m_2 = ((numPoints+1)-i)*subDist;
             P_L(i,1) = (m_1*arr(2,1)+m_2*arr(1,1))/(m_1+m_2); % x-coordinate
             P_L(i,2) = (m_1*arr(2,2)+m_2*arr(1,2))/(m_1+m_2); % y-coordinate
             count = count-1;
             if ~count
                 break;
             end
             P_R(i,1) = (m_1*arr(3,1)+m_2*arr(1,1))/(m_1+m_2);
             P_R(i,2) = (m_1*arr(3,2)+m_2*arr(1,2))/(m_1+m_2);
             count = count-1;
         end
         
         count = size(P_L,1); % I need count of number of rows, so took only 1 column
         for i = 4:num
            if count
               arr(i,:) = P_L(i-3,:); % First fill left arm
               count = count-1;
            else
               arr(i,:) = P_R(i-3-size(P_L,1),:); % Then fill right arm
            end
         end
         
     end
     ft=arr;
end