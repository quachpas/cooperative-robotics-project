
function [uvms] = ComputeTaskReferences(uvms, mission)
	% compute the task references here

	% reference for tool-frame position control task
	[ang, lin] = CartError(uvms.vTg , uvms.vTt);
	uvms.xdot.t = 0.2 * [ang; lin];
	% limit the requested velocities...
	uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 3.0);
	uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 3.0);

	% ASSIGNEMENT 1: updating velocities
	% Reference for vehicle position control task
	[uvms.ang_v, uvms.lin_v] = CartError(uvms.wTgv , uvms.wTv);
	uvms.xdot.v_l = 1.0 * uvms.lin_v;
	uvms.xdot.v_a = 1.0 * uvms.ang_v;
	% limit the requested velocities...
	uvms.xdot.v_l = Saturate(uvms.xdot.v_l, 1.0);
	uvms.xdot.v_a = Saturate(uvms.xdot.v_a, 1.0);
	% END ASSIGNEMENT 1

	% ASSIGNEMENT 2
	uvms.xdot.ha = 0.2 * (0 - norm(uvms.v_rho_ha));
	% END ASSIGNEMENT 2

	% ASSIGNEMENT 3
	uvms.xdot.ma = 0.2 * (uvms.ma - uvms.a) + 1;
	% END ASSIGNEMENT 3

	% ASSIGNEMENT 4
	uvms.xdot.a = 1.0  * (0 - uvms.a);
	% END ASSIGNEMENT 4

	% ASSIGNEMENT 5
	uvms.xdot.und = uvms.p_dot;
	uvms.xdot.vcv = uvms.p_dot;
	% END ASSIGNEMENT 5

    % ASSIGNEMENT 7
	uvms.xdot.at = 1.0 * (0 - norm(uvms.v_rho_at)) - 0.2;
	% END ASSIGNEMENT 7

    % ASSIGNEMENT 8
    uvms.xdot.jl = 1.0 * ((uvms.jlmin + uvms.jlmax) / 2 - uvms.q);
	% END ASSIGNEMENT 8

    % ASSIGNEMENT 9
    uvms.xdot.vnv = zeros(6,1);
    % END ASSIGNEMENT 9

    uvms.xdot.mu = 0.2 * (1 - uvms.mu);

    uvms.xdot.pas = 0.2 * (uvms.pas - uvms.q);