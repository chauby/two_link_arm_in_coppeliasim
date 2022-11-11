function [PVT_point] = getPVTValue(PVT_param_queue, PVT_queue, t)
    t_start = PVT_queue(1,3);
    t_end = PVT_queue(end, 3);
    PVT_point = zeros(1, 3);

    if t <= t_start
        PVT_point = [PVT_queue(1,1), PVT_queue(1, 2), t];
        return;
    else if t >= t_end
        PVT_point = [PVT_queue(end,1), PVT_queue(end, 2), t];
        return;
    else
        for index = 2: size(PVT_queue, 1)
            if t < PVT_queue(index, 3)
                a = PVT_param_queue(index-1, 1);
                b = PVT_param_queue(index-1, 2);
                c = PVT_param_queue(index-1, 3);
                d = PVT_param_queue(index-1, 4);
                t0 = PVT_queue(index-1, 3);

                p = a*(t-t0)^3 + b*(t-t0)^2 + c*(t-t0) + d;
                v = 3*a*(t-t0)^2 + 2*b*(t-t0) + c;
                PVT_point = [p, v, t];
                break;
            end
        end
    end
end
