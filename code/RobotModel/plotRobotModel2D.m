global fig_robot_model;
if (0 == exist('fig_robot_model', 'var'))
    fig_robot_model = gca;
end

global uLINK
global ID

% body
head_pos = uLINK(ID.BODY).p + uLINK(ID.BODY).R*[0, 0, 0.5]';
robotLink.Body = plot(fig_robot_model, [uLINK(ID.BODY).p(1) head_pos(1)], [uLINK(ID.BODY).p(3) head_pos(3)], 'linewidth', 8, 'color', 'm');
robotLink.COM = plot(fig_robot_model, head_pos(1), head_pos(3), 'r.', 'markersize', 30);
robotLink.L_Waist = 0;
robotLink.R_Waist = 0;

%left leg
robotLink.L_Thigh = plot(fig_robot_model, [uLINK(ID.L_LEG_J2).p(1) uLINK(ID.L_LEG_J3).p(1)], [uLINK(ID.L_LEG_J2).p(3) uLINK(ID.L_LEG_J3).p(3)], 'linewidth', 4, 'color', 'k');
robotLink.L_Shank = plot(fig_robot_model, [uLINK(ID.L_LEG_J3).p(1) uLINK(ID.L_LEG_J4).p(1)], [uLINK(ID.L_LEG_J3).p(3) uLINK(ID.L_LEG_J4).p(3)], 'linewidth', 3, 'color', 'k');
robotLink.L_Hip = plot(fig_robot_model, uLINK(ID.L_LEG_J2).p(1), uLINK(ID.L_LEG_J2).p(3), 'm.', 'markersize', 20);
robotLink.L_Knee = plot(fig_robot_model, uLINK(ID.L_LEG_J3).p(1), uLINK(ID.L_LEG_J3).p(3), 'm.', 'markersize', 16);
robotLink.L_Ankle = plot(fig_robot_model, uLINK(ID.L_LEG_J4).p(1), uLINK(ID.L_LEG_J4).p(3), 'r.', 'markersize', 8);

foot_length = 0.15;
robotLink.left_foot_p = uLINK(ID.L_LEG_J4).p + uLINK(ID.L_LEG_J4).R*[foot_length, 0, 0]';
robotLink.L_FOOT = plot(fig_robot_model, [uLINK(ID.L_LEG_J4).p(1),robotLink.left_foot_p(1)],[uLINK(ID.L_LEG_J4).p(3),robotLink.left_foot_p(3)],'linewidth',2,'color','k');

% right leg
robotLink.R_Thigh = plot(fig_robot_model, [uLINK(ID.R_LEG_J2).p(1) uLINK(ID.R_LEG_J3).p(1)], [uLINK(ID.R_LEG_J2).p(3) uLINK(ID.R_LEG_J3).p(3)], 'linewidth', 4, 'color', 'b');
robotLink.R_Shank = plot(fig_robot_model, [uLINK(ID.R_LEG_J3).p(1) uLINK(ID.R_LEG_J4).p(1)], [uLINK(ID.R_LEG_J3).p(3) uLINK(ID.R_LEG_J4).p(3)], 'linewidth', 3, 'color', 'b');
robotLink.R_Hip = plot(fig_robot_model, uLINK(ID.R_LEG_J2).p(1), uLINK(ID.R_LEG_J2).p(3), 'm.', 'markersize', 20);
robotLink.R_Knee = plot(fig_robot_model, uLINK(ID.R_LEG_J3).p(1), uLINK(ID.R_LEG_J3).p(3), 'm.', 'markersize', 16);
robotLink.R_Ankle = plot(fig_robot_model, uLINK(ID.R_LEG_J4).p(1), uLINK(ID.R_LEG_J4).p(3), 'r.', 'markersize', 8);

robotLink.right_foot_p = uLINK(ID.R_LEG_J4).p + uLINK(ID.R_LEG_J4).R*[foot_length, 0, 0]';
robotLink.R_FOOT = plot(fig_robot_model, [uLINK(ID.R_LEG_J4).p(1),robotLink.right_foot_p(1)],[uLINK(ID.R_LEG_J4).p(3),robotLink.right_foot_p(3)],'linewidth',2,'color','k');
