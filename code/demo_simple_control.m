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

desired_q = [0, 0];

data_len = 500;
for i=1:data_len
    desired_q(1) = sin(i*dt);
    desired_q(2) = -sin(i*dt);

    arm_sim_model.setJointAngle('joint_1', desired_q(1));
    arm_sim_model.setJointAngle('joint_2', desired_q(2));

    arm_sim_model.step();
end

arm_sim_model.stopSimulation();
arm_sim_model.shutdownSimulation();

fprintf('Program end\n');
