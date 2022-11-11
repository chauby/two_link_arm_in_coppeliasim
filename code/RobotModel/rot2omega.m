function lin_R = rot2omega(R)
    % Check if R is an identify matrix
    tmp = R - eye(3);
    for i=1:3
        for j=1:3
            if tmp(i, j) >= 1E-6 % R is not an identify matrix
                    theta = acos((R(1, 1) + R(2, 2) + R(3, 3) - 1)/2);
                    lin_R = (theta/(2*sin(theta)))*[R(3,2) - R(2,3), R(1,3) - R(3,1), R(2,1) - R(1,2)]';
                return
            end
        end
    end

    lin_R = [0 0 0]';

end