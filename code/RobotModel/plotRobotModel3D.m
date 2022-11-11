global uLINK;
global ID;

r=0.02;
n=20;

body_color='m';
joint_color='r';
left_leg_color = 'k';
right_leg_color = 'b';

closed=1;
lines=0;  % 0 for only surface, 1 for displaying the line segments on the cylinder

robot_joint_length = [0, 0.02, 0]';

% body
[Xs, Ys, Zs] = sphere;
r_sphere = 0.06;
head_pos = uLINK(ID.BODY).p + uLINK(ID.BODY).R*[0, 0, 0.5]';
robotLink.COM = surf(r_sphere*(Xs + head_pos(1)), r_sphere*(Ys + head_pos(2)), r_sphere*(Zs + head_pos(3)));
robotLink.Body = cylinder3(uLINK(ID.BODY).p, head_pos, 2*r, n, body_color, closed, lines);

robotLink.L_Waist = cylinder3(uLINK(ID.BODY).p, uLINK(ID.L_LEG_J0).p, 2*r, n, body_color, closed, lines);
robotLink.R_Waist = cylinder3(uLINK(ID.BODY).p, uLINK(ID.R_LEG_J0).p, 2*r, n, body_color, closed, lines);
robot_foot_p_loc = [0.2, 0, 0]'; % set foot length

% left leg
robotLink.L_Thigh = cylinder3(uLINK(ID.L_LEG_J2).p, uLINK(ID.L_LEG_J3).p, r, n, left_leg_color, closed, lines);
robotLink.L_Shank = cylinder3(uLINK(ID.L_LEG_J3).p, uLINK(ID.L_LEG_J4).p, r, n, left_leg_color, closed, lines);
robotLink.L_Foot = cylinder3(uLINK(ID.L_LEG_J5).p, uLINK(ID.L_LEG_J5).p + uLINK(ID.L_LEG_J5).R*robot_foot_p_loc, r, n, left_leg_color, closed, lines);
robotLink.L_Hip = cylinder3(uLINK(ID.L_LEG_J2).p + robot_joint_length, uLINK(ID.L_LEG_J2).p - robot_joint_length, r, n, joint_color, closed, lines);
robotLink.L_Knee = cylinder3(uLINK(ID.L_LEG_J3).p + robot_joint_length, uLINK(ID.L_LEG_J3).p - robot_joint_length, r, n, joint_color, closed, lines);
robotLink.L_Ankle = cylinder3(uLINK(ID.L_LEG_J4).p + robot_joint_length, uLINK(ID.L_LEG_J4).p - robot_joint_length, r, n, joint_color, closed, lines);

% right leg
robotLink.R_Thigh = cylinder3(uLINK(ID.R_LEG_J2).p, uLINK(ID.R_LEG_J3).p, r, n, right_leg_color, closed, lines);
robotLink.R_Shank = cylinder3(uLINK(ID.R_LEG_J3).p, uLINK(ID.R_LEG_J4).p, r, n, right_leg_color, closed, lines);
robotLink.R_Foot = cylinder3(uLINK(ID.R_LEG_J5).p, uLINK(ID.R_LEG_J5).p + uLINK(ID.R_LEG_J5).R*robot_foot_p_loc, r, n, right_leg_color, closed, lines);
robotLink.R_Hip = cylinder3(uLINK(ID.R_LEG_J2).p + robot_joint_length, uLINK(ID.R_LEG_J2).p - robot_joint_length, r, n,  joint_color, closed, lines);
robotLink.R_Knee = cylinder3(uLINK(ID.R_LEG_J3).p + robot_joint_length, uLINK(ID.R_LEG_J3).p - robot_joint_length, r, n, joint_color, closed, lines);
robotLink.R_Ankle = cylinder3(uLINK(ID.R_LEG_J4).p + robot_joint_length, uLINK(ID.R_LEG_J4).p - robot_joint_length, r, n, joint_color, closed, lines);
