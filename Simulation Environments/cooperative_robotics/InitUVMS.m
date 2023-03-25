function [uvms] = InitUVMS(robotname)

	% uvms.vTb
	% transformation matrix betwene the arm base wrt vehicle frame
	% expresses how the base of the arm is attached to the vehicle
	% do NOT change, since it must be coherent with the visualization tool
	if (strcmp(robotname, 'DexROV'))
		% do NOT change
		uvms.vTb = [rotation(pi, 0, pi) [0.167 0 -0.43]'; 0 0 0 1];
	else
		if (strcmp(robotname, 'Robust'))
			% do NOT change
			uvms.vTb = [rotation(0, 0, pi) [0.85 0 -0.42]'; 0 0 0 1];
		end
	end
    
    uvms.robotname = robotname;

	uvms.q_dot = [0 0 0 0 0 0 0]';
	uvms.p_dot = [0 0 0 0 0 0]';

	% joint limits corresponding to the actual MARIS arm configuration
	uvms.jlmin  = [-2.9;-1.6;-2.9;-2.95;-2.9;-1.65;-2.8];
	uvms.jlmax  = [2.9;1.65;2.9;0.01;2.9;1.25;2.8];

	% to be computed at each time step
	uvms.wTv = eye(4,4);
	uvms.wTt = eye(4,4);
	uvms.vTw = eye(4,4);
	uvms.vTe = eye(4,4);
	uvms.vTt = eye(4,4);
	uvms.vTg = eye(4,4);
	uvms.Ste = eye(6,6);
	uvms.bTe = eye(4,4);
	uvms.bJe = eye(6,7);
	uvms.djdq = zeros(6,7,7);
	uvms.mu  = 0;
	uvms.phi = zeros(3,1);
	uvms.sensorDistance = 0;

	uvms.Jjl = [];
	uvms.Jmu = [];
	uvms.Jha = [];
	uvms.Jt_a = [];
	uvms.Jt_v = [];
	uvms.Jt = [];
	% ASSIGNEMENT 1
	uvms.Jv_l = [];
	uvms.Jv_a = [];
	% END ASSIGNEMENT 1
	% ASSIGNEMENT 2
	uvms.Jv_ha = [];
	% END ASSIGNEMENT 2
	% ASSIGNEMENT 3
	uvms.Jma = [];
	% END ASSIGNEMENT 3
	% ASSIGNEMENT 4
	uvms.Ja = [];
	% END ASSIGNEMENT 4
	% ASSIGNEMENT 5
	uvms.Jund = [];
	uvms.Jvcv = [];
	% END ASSIGNEMENT 5
    % ASSIGNEMENT 7
    uvms.Jat = [];
    % END ASSIGNEMENT 7
    % ASSIGNEMENT 9
	uvms.Jvnv = [];
	% END ASSIGNEMENT 9
    uvms.Jpas = [];

	uvms.xdot.jl = [];
	uvms.xdot.mu = [];
	uvms.xdot.ha = [];
	uvms.xdot.t = [];
	% ASSIGNEMENT 1
	uvms.x_l = [];
	uvms.x_a = [];
	% END ASSIGNEMENT 1
	% ASSIGNEMENT 2
	uvms.x_ha = [];
	% END ASSIGNEMENT 2
	% ASSIGNEMENT 3
	uvms.xdot.ma = [];
	% END ASSIGNEMENT 3
	% ASSIGNEMENT 4
	uvms.xdot.a = [];
	% END ASSIGNEMENT 4
	% ASSIGNEMENT 5
	uvms.xdot.und = [];
	uvms.xdot.vcv = [];
	% END ASSIGNEMENT 5
    % ASSIGNEMENT 7
    uvms.xdot.at = [];
    % END ASSIGNEMENT 7
    % ASSIGNEMENT 9
	uvms.xdot.vnv = [];
	% END ASSIGNEMENT 9
    uvms.xdot.pas = [];

	uvms.A.jl = zeros(7,7);
	uvms.A.mu = zeros(1,1);
	uvms.A.t = zeros(6,6);
	% ASSIGNEMENT 1
	uvms.A.v_l = zeros(3);
	uvms.A.v_a = zeros(3);
	% END ASSIGNEMENT 1
	% ASSIGNEMENT 2
	uvms.A.ha = zeros(1,1);
	% END ASSIGNEMENT 2
	% ASSIGNEMENT 3
	uvms.A.ma = zeros(1,1);
	% END ASSIGNEMENT 3
	% ASSIGNEMENT 4
	uvms.A.a = zeros(1,1);
	% END ASSIGNEMENT 4
	% ASSIGNEMENT 5
	uvms.A.und = zeros(6);
	uvms.A.vcv = zeros(6);
	% END ASSIGNEMENT 5
    % ASSIGNEMENT 7
	uvms.A.at = zeros(1,1);
	% END ASSIGNEMENT 7
    % ASSIGNEMENT 9
	uvms.A.vnv = zeros(6,13);
	% END ASSIGNEMENT 9
    uvms.A.pas = zeros(7,7);

	% a(x,p) = a^i(x) a^p(p)
	% ASSIGNEMENT 1
	uvms.Ap.v_l = zeros(3);
	uvms.Ap.v_a = zeros(3);
	% END ASSIGNEMENT 1
	% ASSIGNEMENT 2
	uvms.Ap.ha = zeros(1,1);
	% END ASSIGNEMENT 2
	% ASSIGNEMENT 3
	uvms.Ap.ma = zeros(1,1);
	% END ASSIGNEMENT 3
	% ASSIGNEMENT 4
	uvms.Ap.a = zeros(1,1);
	% END ASSIGNEMENT 4
    % ASSIGNEMENT 5
    uvms.Ap.und = eye(6);
    uvms.Ap.vcv = eye(6);
    % END ASSIGNEMENT 5
    % ASSIGNEMENT 6
    uvms.Ap.t = zeros(6);
    % END ASSIGNEMENT 6
    % ASSIGNEMENT 7
    uvms.Ap.at = zeros(1,1);
    % END ASSIGNEMENT 7
    % ASSIGNEMENT 8
	uvms.Ap.jl = zeros(7,7);
	% END ASSIGNEMENT 8
    % ASSIGNEMENT 9
	uvms.Ap.vnv = zeros(6,6);
	% END ASSIGNEMENT 9
    uvms.Ap.pas = zeros(7,7);

    % ASSIGNEMENT 1
	uvms.ang_v = [];
	uvms.lin_v = [];
	% END ASSIGNEMENT 1
	% ASSIGNEMENT 2
	uvms.v_rho_ha = [];
	uvms.v_n_ha = [];
	% END ASSIGNEMENT 2
	% ASSIGNEMENT 3
	uvms.a = [];
    uvms.ma = 1;
	% END ASSIGNEMENT 3
    % ASSIGNEMENT 7
	uvms.wOt = [12.2025   37.3748  -39.8860]';
	% END ASSIGNEMENT 7
    uvms.pas = [-0.0031 1.2586 0.0128 -1.2460 0 0 0]';
end

