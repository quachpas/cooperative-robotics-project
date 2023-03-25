function [uvms] = ComputeActivationFunctions(uvms, mission)
    switch uvms.robotname
        case 'Robust'
	        switch mission.phase
		        case 1
			        % ASSIGNEMENT 1
			        uvms.Ap.v_l = eye(3);
			        uvms.Ap.v_a = eye(3);
			        % END ASSIGNEMENT 1
			        % ASSIGNEMENT 2
			        uvms.Ap.ha = eye(1,1);
			        % END ASSIGNEMENT 2
			        % ASSIGNEMENT 3
			        uvms.Ap.ma = eye(1,1);
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
	                uvms.Ap.jl = zeros(7);
	                % END ASSIGNEMENT 8
                    % ASSIGNEMENT 9
	                uvms.Ap.vnv = zeros(6);
	                % END ASSIGNEMENT 9
		        case 2
			        % ASSIGNEMENT 1
			        uvms.Ap.v_l = eye(3) * DecreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time); %eye(3);
			        uvms.Ap.v_a = eye(3) * DecreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time); %eye(3);
			        % END ASSIGNEMENT 1
			        % ASSIGNEMENT 2
			        uvms.Ap.ha = eye(1,1);
			        % END ASSIGNEMENT 2
			        % ASSIGNEMENT 3
			        uvms.Ap.ma = DecreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
			        % END ASSIGNEMENT 3
			        % ASSIGNEMENT 4
			        uvms.Ap.a = IncreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time); % zeros(1,1); 
			        % END ASSIGNEMENT 4
			        % ASSIGNEMENT 5
			        uvms.Ap.und = eye(6);
			        uvms.Ap.vcv = eye(6);
			        % END ASSIGNEMENT 5
			        % ASSIGNEMENT 6
			        uvms.Ap.t = zeros(6);
			        % END ASSIGNEMENT 6
                    % ASSIGNEMENT 4
			        uvms.Ap.at = IncreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
			        % END ASSIGNEMENT 4
                    % ASSIGNEMENT 8
	                uvms.Ap.jl = zeros(7);
	                % END ASSIGNEMENT 8
                    % ASSIGNEMENT 9
	                uvms.Ap.vnv = zeros(6);
	                % END ASSIGNEMENT 9
                case 3
                    % ASSIGNEMENT 1
			        uvms.Ap.v_l = zeros(3);
			        uvms.Ap.v_a = zeros(3);
			        % END ASSIGNEMENT 1
			        % ASSIGNEMENT 2
			        uvms.Ap.ha = DecreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
			        % END ASSIGNEMENT 2
			        % ASSIGNEMENT 3
			        uvms.Ap.ma = zeros(1,1);
			        % END ASSIGNEMENT 3
			        % ASSIGNEMENT 4
			        uvms.Ap.a = DecreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
			        % END ASSIGNEMENT 4
			        % ASSIGNEMENT 5
			        uvms.Ap.und = eye(6);
			        uvms.Ap.vcv = eye(6);
			        % END ASSIGNEMENT 5
			        % ASSIGNEMENT 6
			        uvms.Ap.t = eye(6) * IncreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
			        % END ASSIGNEMENT 6
                    % ASSIGNEMENT 4
			        uvms.Ap.at = DecreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
			        % END ASSIGNEMENT 4
                    % ASSIGNEMENT 8
	                uvms.Ap.jl = eye(7) * IncreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
	                % END ASSIGNEMENT 8
                    % ASSIGNEMENT 9
	                uvms.Ap.vnv = eye(6) * IncreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
	                % END ASSIGNEMENT 9
            end
        case 'DexROV'
            switch mission.phase
		        case 1
			        uvms.Ap.v_l = eye(3);
			        uvms.Ap.v_a = eye(3);
			        uvms.Ap.ha = eye(1,1);
			        uvms.Ap.t = zeros(6);
	                uvms.Ap.jl = zeros(7);
                    uvms.Ap.mu = zeros(1);
                    uvms.Ap.pas = zeros(7);
                    uvms.Ap.und = eye(6);
                    uvms.Ap.vcv = eye(6);
                case 2
                    uvms.Ap.v_l = eye(3) * DecreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
			        uvms.Ap.v_a = eye(3) * DecreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
			        uvms.Ap.ha = eye(1,1);
			        uvms.Ap.t = eye(6) * IncreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
	                uvms.Ap.jl = eye(7) * IncreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
                    uvms.Ap.mu = IncreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
                    uvms.Ap.pas = IncreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
                    uvms.Ap.und = eye(6);
                    uvms.Ap.vcv = eye(6);
                case 3
            end
    end

	% compute the activation functions here

	% arm tool position control
	% always active
	uvms.A.t = eye(6) * uvms.Ap.t;

	% ASSIGNEMENT 1: moving to goal position

	% inequality objective
	uvms.A.v_l = IncreasingBellShapedFunction(0, 1, 0, 1, norm(uvms.lin_v)) * uvms.Ap.v_l;
	uvms.A.v_a = IncreasingBellShapedFunction(0, 1, 0, 1, norm(uvms.ang_v)) * uvms.Ap.v_a;
	% END ASSIGNEMENT 1

	% ASSIGNEMENT 2: horizontal alignement
	uvms.A.ha = IncreasingBellShapedFunction(0.1, 0.4, 0, 1, norm(uvms.v_rho_ha)) * uvms.Ap.ha;
	% END ASSIGNEMENT 2

	% ASSIGNEMENT 3: minimum altitude
	uvms.A.ma = DecreasingBellShapedFunction(uvms.ma, uvms.ma + 0.5, 0, 1, uvms.a) * uvms.Ap.ma;
	% END ASSIGNEMENT 3
	% ASSIGNEMENT 4
	uvms.A.a = eye(1) * uvms.Ap.a; % equality
	% END ASSIGNEMENT 4

	% ASSIGNEMENT 5
	uvms.A.und = diag([1 1 0 0 0 0]) * uvms.Ap.und; % always active for x/y
	uvms.A.vcv = eye(6) * uvms.Ap.vcv; % Always active
	% END ASSIGNEMENT 5

    % ASSIGNEMENT 7
    uvms.A.at = IncreasingBellShapedFunction(0.2, 0.4, 0, 1, norm(uvms.v_rho_at)) * uvms.Ap.at;
    % END ASSIGNEMENT 7

    % ASSIGNEMENT 8
    jl_val = [uvms.q - uvms.jlmin uvms.jlmax - uvms.q];
    uvms.A.jl = DecreasingBellShapedFunction(0, 0.5, 0, 1, min(jl_val, [], "all")) * uvms.Ap.jl;
    % END ASSIGNEMENT 8

    % ASSIGNEMENT 9
	uvms.A.vnv = eye(6) * uvms.Ap.vnv;
	% END ASSIGNEMENT 9

    uvms.A.mu = DecreasingBellShapedFunction(0, 1, 0, 1, uvms.mu) * uvms.Ap.mu;

    uvms.A.pas = eye(7,7) * uvms.Ap.pas;
    
