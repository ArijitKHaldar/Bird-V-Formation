% Here I will try to return N coordinates of triangle as array for vertex
% angle as parameter
function ft=trianglecoordinates(num,triangle_vertex)
     arr=zeros(num,2);
     arr=[triangle_vertex(1,1),triangle_vertex(1,2);
            1.3,4.7;
           5,1.76;
           triangle_vertex(2,1),triangle_vertex(2,2);
           triangle_vertex(3,1),triangle_vertex(3,2)];
     ft=arr;
end