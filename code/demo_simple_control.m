clc; clear; close all;

dt = 0.05;
arm_sim_model = ArmSimModel();
arm_sim_model.initializeSimModel(dt);

fprintf('Set arm control mode\n');
arm_sim_model.setJointControlMode('joint_1', 'position');
arm_sim_model.setJointControlMode('joint_2', 'position');

fprintf('Set arm PID control parameters\n');
PID = [0.6, 0.0, 0.0];
arm_sim_model.setJointPID('joint_1', PID);
arm_sim_model.setJointPID('joint_2', PID);

% set initial joint angles
desired_q = [0, 0];
actual_q = [0, 0];
arm_sim_model.setJointAngle('joint_1', desired_q(1));
arm_sim_model.setJointAngle('joint_2', desired_q(2));
arm_sim_model.step();

data_len = 100;
reference_joint_angle = zeros(data_len, 2);
actual_joint_angle = zeros(data_len, 2);

v = 2;
for i=1:data_len
    % get actual joint angles
    actual_q(1) = arm_sim_model.getJointAngle('joint_1');
    actual_q(2) = arm_sim_model.getJointAngle('joint_2');

    % set desired joint angles
    desired_q(1) = sin((i-1)*dt*v);
    desired_q(2) = -sin((i-1)*dt*v);

    arm_sim_model.setJointAngle('joint_1', desired_q(1));
    arm_sim_model.setJointAngle('joint_2', desired_q(2));

    arm_sim_model.step();

    % data record
    reference_joint_angle(i, 1) = desired_q(1);
    reference_joint_angle(i, 2) = desired_q(2);
    actual_joint_angle(i, 1) = actual_q(1);
    actual_joint_angle(i, 2) = actual_q(2);
end

arm_sim_model.stopSimulation();
arm_sim_model.shutdownSimulation();


% plot
figure();
subplot(2, 1, 1);
plot(reference_joint_angle(:, 1), 'g');
hold on;
plot(actual_joint_angle(:, 1), 'r--');
hold off;
title('hip angle');
legend('reference', 'actual');

subplot(2, 1, 2);
plot(reference_joint_angle(:, 2), 'g');
hold on;
plot(actual_joint_angle(:, 2), 'r--');
hold off;
title('knee angle');
legend('reference', 'actual');

fprintf('Program end\n');
