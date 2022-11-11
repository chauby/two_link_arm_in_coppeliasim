function [matrix] = rotZ(theta)

matrix = [cos(theta) -sin(theta) 0; sin(theta), cos(theta) 0; 0 0 1];

end

