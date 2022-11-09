classdef ArmSimModel < handle
    properties
        vrep_sim;
        client_ID;

        joint_1_handle;
        joint_2_handle;

        joint_1_angle;
        joint_2_angle;

    end

    methods
        function self = initializeSimModel(self, dt, joint_angle)
            if nargin < 2
                dt = 0.01;
            end

            if nargin < 3
                joint_angle = [0 0];
            end


            fprintf('Connecting to CoppeliaSim API server\n');

            self.vrep_sim = remApi('remoteApi');
            self.vrep_sim.simxFinish(-1); % close all opened connections
            client_ID = self.vrep_sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);

            try
                client_ID ~= -1;
                fprintf('Connected to remote API server\n');
            catch exception
                fprintf('Failed to connect the remote API server\n');
            end
            
            self.client_ID = client_ID;

            % Set the simulation step size for VREP
            retSetTimeStep = self.vrep_sim.simxSetFloatingParameter(self.client_ID, self.vrep_sim.sim_floatparam_simulation_time_step, dt, self.vrep_sim.simx_opmode_oneshot_wait);
            if retSetTimeStep ~= self.vrep_sim.simx_return_ok
                fprintf('Error: setting time step failed\n');
            else
                fprintf('time step setted!\n');
            end

            % Open synchronous mode
            self.vrep_sim.simxSynchronous(self.client_ID, true);

            % Start simulation
            self.vrep_sim.simxStartSimulation(self.client_ID, self.vrep_sim.simx_opmode_oneshot);
            self.vrep_sim.simxSynchronousTrigger(self.client_ID);  % trigger one simulation step

            % get joint handles for left leg
            [return_code, self.joint_1_handle] = self.vrep_sim.simxGetObjectHandle(self.client_ID, 'joint_1', self.vrep_sim.simx_opmode_blocking);
            if (return_code == self.vrep_sim.simx_return_ok)
                fprintf('get object joint_1 ok.\n');
            end

            [return_code, self.joint_2_handle] = self.vrep_sim.simxGetObjectHandle(self.client_ID, 'joint_2', self.vrep_sim.simx_opmode_blocking);
            if (return_code == self.vrep_sim.simx_return_ok)
                fprintf('get object joint_2 ok.\n');
            end
            
            % Get the joint position
            [return_code, q] = self.vrep_sim.simxGetJointPosition(self.client_ID, self.joint_1_handle, self.vrep_sim.simx_opmode_streaming);
            [return_code, q] = self.vrep_sim.simxGetJointPosition(self.client_ID, self.joint_2_handle, self.vrep_sim.simx_opmode_streaming);

            % Set the initialized position for each joint
            self.vrep_sim.simxSetJointTargetPosition(self.client_ID, self.joint_1_handle, joint_angle(1), self.vrep_sim.simx_opmode_streaming);
            self.vrep_sim.simxSetJointTargetPosition(self.client_ID, self.joint_2_handle, joint_angle(2), self.vrep_sim.simx_opmode_streaming);
        end

        function startSimulation(self)
            self.vrep_sim.simxStartSimulation(self.client_ID, self.vrep_sim.simx_opmode_oneshot);
        end

        function step(self, sended_enable)
            self.vrep_sim.simxSynchronousTrigger(self.client_ID);  % trigger one simulation step
            if((nargin >= 2) && (sended_enable == true))
                self.vrep_sim.simxGetPingTime(self.client_ID); % Make sure that the last command sent out
            end
        end

        function stopSimulation(self)
            self.vrep_sim.simxStopSimulation(self.client_ID, self.vrep_sim.simx_opmode_oneshot);
            self.vrep_sim.simxGetPingTime(self.client_ID); % Make sure that the last command sent out
        end

        function shutdownSimulation(self)
            self.vrep_sim.simxFinish(self.client_ID); % Now close the connection to CoppeliaSim:
            self.vrep_sim.delete(); % call the destructor
        end

        function q = getJointAngle(self, joint_name)
            q = 0;

            if strcmp(joint_name, 'joint_1')
                [return_code, q] = self.vrep_sim.simxGetJointPosition(self.client_ID, self.joint_1_handle, self.vrep_sim.simx_opmode_buffer);
            elseif strcmp(joint_name, 'joint_2')
                [return_code, q] = self.vrep_sim.simxGetJointPosition(self.client_ID, self.joint_2_handle, self.vrep_sim.simx_opmode_buffer);
            else
                fprintf('Error: joint name: %s can not be recognized.\n', joint_name);
            end
        end

        function q = getJointAngles(self)
            q = [0, 0];

            q(1) = self.getJointAngle('joint_1');
            q(2) = self.getJointAngle('joint_2');
        end

        function setJointAngle(self, joint_name, q)
            if strcmp(joint_name, 'joint_1')
                self.vrep_sim.simxSetJointTargetPosition(self.client_ID, self.joint_1_handle, q, self.vrep_sim.simx_opmode_streaming);
            elseif strcmp(joint_name, 'joint_2')
                self.vrep_sim.simxSetJointTargetPosition(self.client_ID, self.joint_2_handle, q, self.vrep_sim.simx_opmode_streaming);
            else
                fprintf('Error: set joint angle failed, can not recognize joint name: %s\n', joint_name);
            end 
        end

        function setAllJointAngles(self, q)
            self.setJointAngle('joint_1', q(1));
            self.setJointAngle('joint_2', q(2));
        end
        
        function setJointControlMode(self, joint_name, mode)
            if mode == 'position'
                control_enable = true;
            elseif mode == 'torque'
                control_enable = false;
            else
                fprintf('Error: set joint control mode error, can not recognize mode: %s\n', mode);
            end

            if strcmp(joint_name, 'joint_1')
                self.vrep_sim.simxSetObjectIntParameter(self.client_ID, self.joint_1_handle, 2001, control_enable, self.vrep_sim.simx_opmode_oneshot);
            elseif strcmp(joint_name, 'joint_2')
                self.vrep_sim.simxSetObjectIntParameter(self.client_ID, self.joint_2_handle, 2001, control_enable, self.vrep_sim.simx_opmode_oneshot);
            else
                fprintf('Error: set joint control mode error, can not recognize joint name: %s\n', joint_name);
            end
        end
        
        function setJointPID(self, joint_name, PID)
            if strcmp(joint_name, 'joint_1')
                self.vrep_sim.simxSetObjectFloatParameter(self.client_ID, self.joint_1_handle, 2002, PID(1), self.vrep_sim.simx_opmode_oneshot);
                self.vrep_sim.simxSetObjectFloatParameter(self.client_ID, self.joint_1_handle, 2003, PID(2), self.vrep_sim.simx_opmode_oneshot);
                self.vrep_sim.simxSetObjectFloatParameter(self.client_ID, self.joint_1_handle, 2004, PID(3), self.vrep_sim.simx_opmode_oneshot);
            elseif strcmp(joint_name, 'joint_2')
                self.vrep_sim.simxSetObjectFloatParameter(self.client_ID, self.joint_2_handle, 2002, PID(1), self.vrep_sim.simx_opmode_oneshot);
                self.vrep_sim.simxSetObjectFloatParameter(self.client_ID, self.joint_2_handle, 2003, PID(2), self.vrep_sim.simx_opmode_oneshot);
                self.vrep_sim.simxSetObjectFloatParameter(self.client_ID, self.joint_2_handle, 2004, PID(3), self.vrep_sim.simx_opmode_oneshot);
            else
                fprintf('Error: set joint PID error, can not recognize joint name: %s\n', joint_name);
            end
        end

        function setJointTorque(self, joint_name, desired_torque)
            if strcmp(joint_name, 'joint_1')
                if desired_torque > 0
                    self.vrep_sim.simxSetJointTargetVelocity(self.client_ID, self.joint_1_handle, 1000, self.vrep_sim.simx_opmode_oneshot);
                    self.vrep_sim.simxSetJointMaxForce(self.client_ID, self.joint_1_handle, abs(desired_torque), self.vrep_sim.simx_opmode_oneshot);
                else
                    self.vrep_sim.simxSetJointTargetVelocity(self.client_ID, self.joint_1_handle, -1000, self.vrep_sim.simx_opmode_oneshot);
                    self.vrep_sim.simxSetJointMaxForce(self.client_ID, self.joint_1_handle, abs(desired_torque), self.vrep_sim.simx_opmode_oneshot);
                end
            elseif strcmp(joint_name, 'joint_2')
                if desired_torque > 0
                    self.vrep_sim.simxSetJointTargetVelocity(self.client_ID, self.joint_2_handle, 1000, self.vrep_sim.simx_opmode_oneshot);
                    self.vrep_sim.simxSetJointMaxForce(self.client_ID, self.joint_2_handle, abs(desired_torque), self.vrep_sim.simx_opmode_oneshot);
                else
                    self.vrep_sim.simxSetJointTargetVelocity(self.client_ID, self.joint_2_handle, -1000, self.vrep_sim.simx_opmode_oneshot);
                    self.vrep_sim.simxSetJointMaxForce(self.client_ID, self.joint_2_handle, abs(desired_torque), self.vrep_sim.simx_opmode_oneshot);
                end
            else
                fprintf('Error: set joint torque error, can not recognize joint name: %s\n', joint_name);
            end
        end

    end
end