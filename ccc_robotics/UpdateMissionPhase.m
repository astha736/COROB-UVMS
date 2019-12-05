function [uvms, mission] = UpdateMissionPhase(uvms, mission)

% mission.phase = 1; % way-point navigation,  position control
% mission.phase_time = 0;
% 
% mission.ea.mu = 1;
% mission.ea.t = zeros(6,6);
% mission.ea.ha = eye(6);
% mission.ea.poc = eye(6);
% mission.ea.mac = eye(6,6);
% mission.ea.la = zeros(6,6);

if (all(uvms.totalError) == 0)
    return
end 

% here we write the logic to change phase 
    switch mission.phase
        case 0
            % this is indication that everything is done
        case 1  
            if (all((uvms.totalError) < 0.1)==1)
                disp('change in phase*****************************************');
                mission.phase = 2;
                mission.ea.poc = zeros(6,6);
                mission.ea.mac = zeros(6,6);
                mission.ea.la  = eye(6);  
            end
        case 2
            if( uvms.mac.wdispf < 0.2)
                mission.ea.la  = zeros(6,6); 
                mission.phase = 0;
            end
           
        case 3 
            
        
     
    end
    
end

