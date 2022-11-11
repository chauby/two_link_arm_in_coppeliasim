function J = calculateJacobian(idx)
    global uLINK
    j_size = length(idx);
    target = uLINK(idx(end)).p;
    J = zeros(6, j_size);

    for n = 1: j_size
        j = idx(n);
        a = uLINK(j).R * uLINK(j).a;
        J(:, n) = [cross(a, target - uLINK(j).p); a];
    end

end
