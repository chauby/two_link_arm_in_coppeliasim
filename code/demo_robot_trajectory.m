close all; clear; clc;

%% trajectory generation
t_given = [-0.6, -0.3, 0, 0.2, 0.6]';
p_given = [-0.6, -0.3, -0.5, -0.2, -0.4]';
v_given = [0, 0, 0, 0, 0]';

PVT_queue = [p_given, v_given, t_given];
PVT_param_queue = PVTSpline(PVT_queue);

data_len = 200;
PVT_spline = zeros(data_len, 3);
t = linspace(t_given(1), t_given(end), data_len)';
for index=1:data_len
    PVT_spline(index, :) = getPVTValue(PVT_param_queue, PVT_queue, t(index));
end


% initialize arm simulation model
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

% create dummy for key point display
[return_code, dummy_1_handle] = arm_sim_model.vrep_sim.simxCreateDummy(arm_sim_model.client_ID, 0.05, [], arm_sim_model.vrep_sim.simx_opmode_blocking);
[return_code, dummy_2_handle] = arm_sim_model.vrep_sim.simxCreateDummy(arm_sim_model.client_ID, 0.05, [], arm_sim_model.vrep_sim.simx_opmode_blocking);
[return_code, dummy_3_handle] = arm_sim_model.vrep_sim.simxCreateDummy(arm_sim_model.client_ID, 0.05, [], arm_sim_model.vrep_sim.simx_opmode_blocking);
[return_code, dummy_4_handle] = arm_sim_model.vrep_sim.simxCreateDummy(arm_sim_model.client_ID, 0.05, [], arm_sim_model.vrep_sim.simx_opmode_blocking);
[return_code, dummy_5_handle] = arm_sim_model.vrep_sim.simxCreateDummy(arm_sim_model.client_ID, 0.05, [], arm_sim_model.vrep_sim.simx_opmode_blocking);

% set dummy position
[return_code, base_position] = arm_sim_model.vrep_sim.simxGetObjectPosition(arm_sim_model.client_ID, arm_sim_model.joint_1_handle, -1, arm_sim_model.vrep_sim.simx_opmode_blocking);

arm_sim_model.vrep_sim.simxSetObjectPosition(arm_sim_model.client_ID, dummy_1_handle, -1, [base_position(1) + t_given(1), 0, base_position(3) + p_given(1)]', arm_sim_model.vrep_sim.simx_opmode_oneshot);
arm_sim_model.vrep_sim.simxSetObjectPosition(arm_sim_model.client_ID, dummy_2_handle, -1, [base_position(1) + t_given(2), 0, base_position(3) + p_given(2)]', arm_sim_model.vrep_sim.simx_opmode_oneshot);
arm_sim_model.vrep_sim.simxSetObjectPosition(arm_sim_model.client_ID, dummy_3_handle, -1, [base_position(1) + t_given(3), 0, base_position(3) + p_given(3)]', arm_sim_model.vrep_sim.simx_opmode_oneshot);
arm_sim_model.vrep_sim.simxSetObjectPosition(arm_sim_model.client_ID, dummy_4_handle, -1, [base_position(1) + t_given(4), 0, base_position(3) + p_given(4)]', arm_sim_model.vrep_sim.simx_opmode_oneshot);
arm_sim_model.vrep_sim.simxSetObjectPosition(arm_sim_model.client_ID, dummy_5_handle, -1, [base_position(1) + t_given(5), 0, base_position(3) + p_given(5)]', arm_sim_model.vrep_sim.simx_opmode_oneshot);


%% initialize robot kinematics model
global uLINK;
RobotModelInitialization; % create the robot model
FK_leg(ID.BODY); % forward kinematics calculation

%% go to the first position
fprintf('Go to the first place \n');
target.p = [PVT_spline(1,3), RobotWaistWidth, PVT_spline(1,1)]';
target.R = rotX(0);

uLINK(ID.L_LEG_J2).q = 0;
uLINK(ID.L_LEG_J3).q = 0.5;
uLINK(ID.R_LEG_J2).q = 0;
uLINK(ID.R_LEG_J3).q = 0.5;
FK_leg(ID.BODY);

IK_leg_Jacobian(ID.L_LEG_J5, target);
FK_leg(ID.BODY);

actual_q(1) = arm_sim_model.getJointAngle('joint_1');
actual_q(2) = arm_sim_model.getJointAngle('joint_2');

target_q_1 = uLINK(ID.L_LEG_J2).q;
target_q_2 = uLINK(ID.L_LEG_J3).q;

first_time = 50;
for i=1:first_time
    desired_q(1) = i*(target_q_1 - actual_q(1))/first_time;
    desired_q(2) = i*(target_q_2 - actual_q(2))/first_time;

    arm_sim_model.setJointAngle('joint_1', desired_q(1));
    arm_sim_model.setJointAngle('joint_2', desired_q(2));

    arm_sim_model.step();
end


%% main loop
fprintf('Go along the trajectory \n');

% get end effector handle in coppeliaSim 
[return_code, end_effector_handle] = arm_sim_model.vrep_sim.simxGetObjectHandle(arm_sim_model.client_ID, 'tip', arm_sim_model.vrep_sim.simx_opmode_blocking);

reference_end_effector_trajectory_record = zeros(3, data_len);
actual_end_effector_trajectory_record = zeros(3, data_len);

for i=1:data_len
    target.p = [PVT_spline(i,3), RobotWaistWidth, PVT_spline(i,1)]';
    target.R = rotX(0);
    IK_leg_Jacobian(ID.L_LEG_J5, target);
    FK_leg(ID.BODY);

    % get actual joint angles
    actual_q(1) = arm_sim_model.getJointAngle('joint_1');
    actual_q(2) = arm_sim_model.getJointAngle('joint_2');

    % get actual end effector position
    [return_code, end_effector_position] = arm_sim_model.vrep_sim.simxGetObjectPosition(arm_sim_model.client_ID, end_effector_handle, -1, arm_sim_model.vrep_sim.simx_opmode_blocking);

    % set desired joint angles
    desired_q(1) = uLINK(ID.L_LEG_J2).q;
    desired_q(2) = uLINK(ID.L_LEG_J3).q;

    arm_sim_model.setJointAngle('joint_1', desired_q(1));
    arm_sim_model.setJointAngle('joint_2', desired_q(2));

    arm_sim_model.step();

    % record foot trajectory for plot
    reference_end_effector_trajectory_record(:, i) = uLINK(ID.L_LEG_J4).p;
    actual_end_effector_trajectory_record(:, i) = end_effector_position - base_position;
end

arm_sim_model.stopSimulation();
arm_sim_model.shutdownSimulation();

% plot
figure();
plot(reference_end_effector_trajectory_record(1,:), reference_end_effector_trajectory_record(3, :), 'r');
hold on;
plot(actual_end_effector_trajectory_record(1,:), actual_end_effector_trajectory_record(3, :), 'g--');
hold off;
legend('reference', 'actual');
