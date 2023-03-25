% Assignements
% 1: Moving to goal position
% 2: Horizontal alignement
% 3: Minimum altitude
% 4: Altitude
% 5: Vehicle Constrained velocity / Underactuation simulation
% 6: Arm joints update
% 7: Alignement to target control objective
% 8: joint limits
% 9: vehicle null velocity


function MainRobust
	addpath('./simulation_scripts');
	clc;
	clear;
	close all

	% Simulation variables (integration and final time)
	deltat = 0.005;
	end_time = 40;
	loop = 1;
	maxloops = ceil(end_time/deltat);

	% this struct can be used to evolve what the UVMS has to do
	mission.phase = 1;
	mission.phase_time = 0;

	% Rotation matrix to convert coordinates between Unity and the <w> frame
	% do not change
	wuRw = rotation(0,-pi/2,pi/2);
	vRvu = rotation(-pi/2,0,-pi/2);

	% pipe parameters
	u_pipe_center = [-10.66 31.47 -1.94]'; % in unity coordinates
	pipe_center = wuRw'*u_pipe_center;     % in world frame coordinates
	pipe_radius = 0.3;

	% rock position
	rock_center = [12.2025   37.3748  -39.8860]'; % in world frame coordinates

	% UDP Connection with Unity viewer v2
	uArm = udp('127.0.0.1',15000,'OutputDatagramPacketSize',28);
	uVehicle = udp('127.0.0.1',15001,'OutputDatagramPacketSize',24);
	fopen(uVehicle);
	fopen(uArm);
	uAltitude = dsp.UDPReceiver('LocalIPPort',15003,'MessageDataType','single');
	uAltitude.setup();

	% Preallocation
	plt = InitDataPlot(maxloops);

	% initialize uvms structure
	uvms = InitUVMS('Robust');
	% uvms.q
	% Initial joint positions. You can change these values to initialize the simulation with a
	% different starting position for the arm
	uvms.q = [-0.0031 0 0.0128 -1.2460 0.0137 0.0853-pi/2 0.0137]';
	% uvms.p
	% initial position of the vehicle
	% the vector contains the values in the following order
	% [x y z r(rot_x) p(rot_y) y(rot_z)]
	% RPY angles are applied in the following sequence
	% R(rot_x, rot_y, rot_z) = Rz (rot_z) * Ry(rot_y) * Rx(rot_x)
	% uvms.p = [8.5 38.5 -38   0 -0.06 0.5]';
	uvms.p = [8.5 38.5 -36 0 -0.06 0.5]';

	% defines the goal position for the end-effector/tool position task
	% uvms.goalPosition = [12.2025   37.3748  -39.8860]';
	%uvms.goalPosition = [12.2025   37.3748  -39.8860]';
    uvms.goalPosition = rock_center;
	uvms.wRg = rotation(0, pi/2, 0);
	uvms.wTg = [uvms.wRg uvms.goalPosition; 0 0 0 1];

	% defines the goal position for the vehicle position task
	uvms.vehicleGoalPosition = [10.5 37.5 -38]';
	uvms.wRgv = rotation(0, -0.06, 0.5);
	uvms.wTgv = [uvms.wRgv uvms.vehicleGoalPosition; 0 0 0 1];

	% defines the tool control point
	uvms.eTt = eye(4);

	tic
	for t = 0:deltat:end_time
		% update all the involved variables
		uvms = UpdateTransforms(uvms);
		uvms = ComputeJacobians(uvms);
		uvms = ComputeTaskReferences(uvms, mission);
		uvms = ComputeActivationFunctions(uvms, mission);

		% receive altitude information from unity
		uvms = ReceiveUdpPackets(uvms, uAltitude);

		% main kinematic algorithm initialization
		% ydotbar order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
		% the vector of the vehicle linear and angular velocities are assumed
		% projected on <v>

		ydotbar = zeros(13,1);
		Qp = eye(13);
		% add all the other tasks here!
		% the sequence of iCAT_task calls defines the priority

		% TPIK 1 (centralized)
		% ASSIGNEMENT 5
		% [Qp, ydotbar] = iCAT_task(uvms.A.und,    uvms.Jund,    Qp, ydotbar, uvms.xdot.und,  0.0001,   0.01, 10);
		% END ASSIGNMENT 5
        % ASSIGNEMENT 9
		[Qp, ydotbar] = iCAT_task(uvms.A.vnv,    uvms.Jvnv,    Qp, ydotbar, uvms.xdot.vnv,  0.0001,   0.01, 10);
		% END ASSIGNMENT 9
		% ASSIGNEMENT 3
		[Qp, ydotbar] = iCAT_task(uvms.A.ma,    uvms.Jma,    Qp, ydotbar, uvms.xdot.ma,  0.0001,   0.01, 10);
		% END ASSIGNMENT 3
		% ASSIGNEMENT 2
		[Qp, ydotbar] = iCAT_task(uvms.A.ha,    uvms.Jha,    Qp, ydotbar, uvms.xdot.ha,  0.0001,   0.01, 10);
		% END ASSIGNMENT 2
        % ASSIGNEMENT 7
		[Qp, ydotbar] = iCAT_task(uvms.A.at,    uvms.Jat,    Qp, ydotbar, uvms.xdot.at,  0.0001,   0.01, 10);
		% END ASSIGNMENT 7
		% ASSIGNEMENT 4
		[Qp, ydotbar] = iCAT_task(uvms.A.a,    uvms.Ja,    Qp, ydotbar, uvms.xdot.a,  0.0001,   0.01, 10);
		% END ASSIGNMENT 4
        % ASSIGNEMENT 8
		[Qp, ydotbar] = iCAT_task(uvms.A.jl,    uvms.Jjl,    Qp, ydotbar, uvms.xdot.jl,  0.0001,   0.01, 10);
		% END ASSIGNMENT 8
		% ASSIGNEMENT 6
		[Qp, ydotbar] = iCAT_task(uvms.A.t,    uvms.Jt,    Qp, ydotbar, uvms.xdot.t,  0.000001,   0.0001, 10);
		% END ASSIGNMENT 6
		% ASSIGNEMENT 1
		[Qp, ydotbar] = iCAT_task(uvms.A.v_l,    uvms.Jv_l,    Qp, ydotbar, uvms.xdot.v_l,  0.0001,   0.01, 10);
		[Qp, ydotbar] = iCAT_task(uvms.A.v_a,    uvms.Jv_a,    Qp, ydotbar, uvms.xdot.v_a,  0.0001,   0.01, 10);
		% END ASSIGNEMENT 1
		[Qp, ydotbar] = iCAT_task(eye(13),     eye(13),    Qp, ydotbar, zeros(13,1),  0.0001,   0.01, 10);    % this task should be the last one

% 		vehicle_ref_velocity = ydotbar(8:13);
% 
% 		% TPIK 2 (arm optimized on current vehicle velocity)
% 		ydotbar = zeros(13,1);
% 		Qp = eye(13);
% 		% ASSIGNEMENT 5
% 		[Qp, ydotbar] = iCAT_task(uvms.A.vcv,    uvms.Jvcv,    Qp, ydotbar, uvms.xdot.vcv,  0.0001,   0.01, 10);
% 		% END ASSIGNMENT 5
% 	    % ASSIGNEMENT 3
% 		[Qp, ydotbar] = iCAT_task(uvms.A.ma,    uvms.Jma,    Qp, ydotbar, uvms.xdot.ma,  0.0001,   0.01, 10);
% 		% END ASSIGNMENT 3
% 		% ASSIGNEMENT 2
% 		[Qp, ydotbar] = iCAT_task(uvms.A.ha,    uvms.Jha,    Qp, ydotbar, uvms.xdot.ha,  0.0001,   0.01, 10);
% 		% END ASSIGNMENT 2
%         % ASSIGNEMENT 7
% 		[Qp, ydotbar] = iCAT_task(uvms.A.at,    uvms.Jat,    Qp, ydotbar, uvms.xdot.at,  0.0001,   0.01, 10);
% 		% END ASSIGNMENT 7
% 		% ASSIGNEMENT 4
% 		[Qp, ydotbar] = iCAT_task(uvms.A.a,    uvms.Ja,    Qp, ydotbar, uvms.xdot.a,  0.0001,   0.01, 10);
% 		% END ASSIGNMENT 4
% 		% ASSIGNEMENT 6
% 		% [Qp, ydotbar] = iCAT_task(uvms.A.t,    uvms.Jt,    Qp, ydotbar, uvms.xdot.t,  0.000001,   0.0001, 10);
% 		% END ASSIGNMENT 6
% 		% ASSIGNEMENT 1
% 		[Qp, ydotbar] = iCAT_task(uvms.A.v_l,    uvms.Jv_l,    Qp, ydotbar, uvms.xdot.v_l,  0.0001,   0.01, 10);
% 		[Qp, ydotbar] = iCAT_task(uvms.A.v_a,    uvms.Jv_a,    Qp, ydotbar, uvms.xdot.v_a,  0.0001,   0.01, 10);
% 		% END ASSIGNEMENT 1
% 
%       [Qp, ydotbar] = iCAT_task(eye(13),     eye(13),    Qp, ydotbar, zeros(13,1),  0.0001,   0.01, 10);    % this task should be the last one
% 
		% get the two variables for integration
		% USE TPIK 2 for EE optimization
		uvms.q_dot = ydotbar(1:7);
		% USE TPIK 1 FOR VEHICLE REFERENCE VELOCITY
        uvms.p_dot = ydotbar(8:13);
		% uvms.p_dot = vehicle_ref_velocity;

		% Simulate Underactuation: override desired angular velocity w_x
		% uvms.p_dot(4) = uvms.p_dot(4) + 0.2*sin(2*pi*0.5*t);

		% Integration
		uvms.q = uvms.q + uvms.q_dot*deltat;
		% beware: p_dot should be projected on <v>
		uvms.p = integrate_vehicle(uvms.p, uvms.p_dot, deltat);

		% check if the mission phase should be changed
		[uvms, mission] = UpdateMissionPhase(uvms, mission);

		% send packets to Unity viewer
		SendUdpPackets(uvms,wuRw,vRvu,uArm,uVehicle);

		% collect data for plots
		plt = UpdateDataPlot(plt,uvms,t,loop);
		loop = loop + 1;

		% add debug prints here
		if (mod(t,0.1) == 0)
			disp(t)
            disp(uvms.q)
            disp(uvms.jlmin)
            disp(uvms.jlmax)
            disp(uvms.q - uvms.jlmin)
            disp(uvms.jlmax - uvms.q)
            disp(uvms.wTt(1:3,4) - rock_center)
			%         altitiude = uvms.a
			%         min_alt_ap = uvms.Ap.ma
			%         alt_ap = uvms.Ap.a
			%         uvms.sensorDistance
			%         uvms.A.ha
			%
			%         reference_tool_velocity = uvms.xdot.t'
			%         ideal_tool_velocity = (uvms.Jt*ydotbar)'
			%         actual_tool_velocity = (uvms.Jt*[uvms.q_dot; uvms.p_dot])'
		end

		mission.phase_time = mission.phase_time + deltat;

		% enable this to have the simulation approximately evolving like real
		% time. Remove to go as fast as possible
		%SlowdownToRealtime(deltat);
	end

	fclose(uVehicle);
	fclose(uArm);

	PrintPlot(plt);

end
