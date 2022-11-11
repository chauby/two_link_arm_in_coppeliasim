function plotRobotModel(view_mode)
    if view_mode == 2
        view(2);
        plotRobotModel2D;
    elseif view_mode == 3
        view(3);
        plotRobotModel3D;
    else
        error('the view_mode can only be 2 or 3');
    end
    pause(0.002);
end