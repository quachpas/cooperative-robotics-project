function [uvms, mission] = UpdateMissionPhase(uvms, mission)
switch uvms.robotname
    case 'Robust'
        switch mission.phase
		    case 1
			    % add policy for changing phase
			    [ang_e, lin_e] = CartError(uvms.wTgv, uvms.wTv);
			    if (norm(ang_e) < 1.0 && norm(lin_e) < 1.0)
				    mission.phase = 2;
				    mission.phase_time = 0;
				    disp("change to phase 2")
			    end
		    case 2
                if (uvms.a < 0.1)
                    mission.phase = 3;
                    mission.phase_time = 0;
                    disp("change to phase 3");
                end
        end
    case 'DexROV'
        switch mission.phase
		    case 1
                % add policy for changing phase
			    [ang_e, lin_e] = CartError(uvms.wTgv, uvms.wTv);
			    if (norm(ang_e) < 0.4 && norm(lin_e) < 0.5)
				    mission.phase = 2;
				    mission.phase_time = 0;
				    disp("change to phase 2")
			    end
            case 2
	end
end

