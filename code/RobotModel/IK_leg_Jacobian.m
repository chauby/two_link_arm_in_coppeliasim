function IK_leg_Jacobian(to, target)
    global uLINK

    lambda = 0.2;
    FK_leg(1);
    idx = findRoute(to);
    for n = 1:16
        J = calculateJacobian(idx);
        err = calculateVWerr(target, uLINK(to));

        if norm(err) < 1E-6
            return;
        end

        dq = lambda*(J\err);

        for nn = 1:length(idx)
            j = idx(nn);
            uLINK(j).q = uLINK(j).q + dq(nn);
        end

        FK_leg(1);
    end
end