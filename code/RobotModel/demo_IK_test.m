close all;
global uLINK;

RobotModelInitialization; % create the robot model
FK_leg(ID.BODY); % forward kinematics calculation
TotalMass(ID.BODY);

global fig_robot_model;
fRobotModel = figure;
set(fRobotModel, 'position', [1000,200, 500, 500]);
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
if view_flag == 2
    view(2); % 2D view
elseif view_flag == 3
    view(3); % 3D view
else
    error('view_flag can be either 2 or 3')
end

% plot a box
data_point_num = 50;

x1=-0.4;
x2=0.6;
z1=-0.6;
z2=0.6;

r = 0.6;

x=[linspace(x1, x2, data_point_num)'; ones(data_point_num, 1) * x2;  linspace(x2, x1, data_point_num)'; ones(data_point_num, 1)*x1];
z=[ones(data_point_num, 1)*z1; linspace(z1, z2, data_point_num)'; ones(data_point_num,1)*z2; linspace(z2, z1, data_point_num)'];

% plot a circle
x3=linspace(x1, x2, 2*data_point_num)';
z3=-sqrt(r^2 - x3.^2);

x4=linspace(x2, x1, 2*data_point_num)';
z4=sqrt(r^2 - x4.^2);

x5=[x3; x4];
z5=[z3; z4];

left_foot_trajectory_record = [];
right_foot_trajectory_record = [];

for i=1:length(x)
    uLINK(ID.L_LEG_J4).p = [x(i), RobotWaistWidth, z(i)]';
    LeftJointQ =IK_leg(uLINK(ID.BODY),RobotWaistWidth,RobotThighLength, RobotShankLength,uLINK(ID.L_LEG_J4));
    uLINK(ID.L_LEG_J2).q = LeftJointQ(3);
    uLINK(ID.L_LEG_J3).q = LeftJointQ(4);

    uLINK(ID.R_LEG_J4).p = [x5(i), -RobotWaistWidth, z5(i)]';
    RightJointQ =IK_leg(uLINK(ID.BODY),-RobotWaistWidth,RobotThighLength, RobotShankLength,uLINK(ID.R_LEG_J4));
    uLINK(ID.R_LEG_J2).q = RightJointQ(3);
    uLINK(ID.R_LEG_J3).q = RightJointQ(4);

    FK_leg(ID.BODY);

    % record foot trajectory for plot
    left_foot_trajectory_record = [left_foot_trajectory_record, uLINK(ID.L_LEG_J4).p];
    right_foot_trajectory_record = [right_foot_trajectory_record, uLINK(ID.R_LEG_J4).p];

    cla(fig_robot_model);
    if view_flag == 2
        plotRobotModel2D;
        plot(left_foot_trajectory_record(1, 1:i), left_foot_trajectory_record(3, 1:i), 'b');
        plot(right_foot_trajectory_record(1, 1:i), right_foot_trajectory_record(3, 1:i), 'k');
    else
        plotRobotModel3D;
        plot3(left_foot_trajectory_record(1, 1:i), left_foot_trajectory_record(2, 1:i), left_foot_trajectory_record(3, 1:i), 'b');
        plot3(right_foot_trajectory_record(1, 1:i), right_foot_trajectory_record(2, 1:i), right_foot_trajectory_record(3, 1:i), 'k');
    end

    drawnow; % draw figures now
end
hold off;
