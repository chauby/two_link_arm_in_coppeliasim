function [PVT_param_queue] = PVTSpline(PVT_queue)
    queue_len = size(PVT_queue, 1);
    if queue_len < 2
        error('point num too small');
    end

    PVT_param_queue = zeros(queue_len-1, 4);

    for index =1:queue_len-1
        PVT_param_queue(index,:) = PVTSpline2Points(PVT_queue(index,:), PVT_queue(index+1,:));
    end
end

