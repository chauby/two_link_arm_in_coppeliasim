clear all;
close all;
global uLINK;
RobotModelInitialization; % create the robot model
FK_leg(ID.BODY); % forward kinematics calculation
TotalMass(ID.BODY);

uLINK(ID.L_LEG_J2).q = 0;
uLINK(ID.L_LEG_J3).q = 0.5;
uLINK(ID.R_LEG_J2).q = 0;
uLINK(ID.R_LEG_J3).q = 0.5;

FK_leg(ID.BODY);

% ============ test the robot model =============
global fig_robot_model;
fRobotModel = figure;
set(fRobotModel, 'position', [1000, 200, 500, 500]);
axis equal;
xlim([-1,1]);
ylim([-1,1]);
zlim([-1, 1]);
title('Robot Model Test');
xlabel('x');
ylabel('y');
grid on;
hold on;
fig_robot_model = gca;

view_flag = 2; % 2 for 2D view, 3 for 3D view
if view_flag == 2 % 2D view
    view(2);
elseif view_flag == 3 % 3D view
    % view(3);
    view(28, 14);
else
    error('view_flag can be either 2 or 3')
end


% plot a circle
r = 0.2;
center = [0, -0.5];
theta = linspace(-pi/2, 3*pi/2, 100);
x = center(1) + r*cos(theta);
z = center(2) + r*sin(theta);

left_foot_trajectory_record = [];

for i=1:length(x)
    target.p = [x(i), RobotWaistWidth, z(i)]';
    target.R = rotX(0);
    IK_leg_Jacobian(ID.L_LEG_J5, target);
    J_left_joint_q(i,:) = [uLINK(ID.L_LEG_J2).q, uLINK(ID.L_LEG_J3).q];

    target.p = [x(i), -RobotWaistWidth, z(i)]';
    target.R = rotX(0);
    IK_leg_Jacobian(ID.R_LEG_J5, target);
    J_right_joint_q(i,:) = [uLINK(ID.R_LEG_J2).q, uLINK(ID.R_LEG_J3).q];

    % record foot trajectory for plot
    left_foot_trajectory_record = [left_foot_trajectory_record, target.p];

    cla(fig_robot_model);
    if view_flag == 2
        plotRobotModel2D;
        plot(left_foot_trajectory_record(1, 1:i), left_foot_trajectory_record(3, 1:i), 'b');
    else
        plotRobotModel3D;
        plot3(left_foot_trajectory_record(1, 1:i), left_foot_trajectory_record(2, 1:i), left_foot_trajectory_record(3, 1:i), 'b');
    end

    drawnow; % draw figures now
end
hold off;
