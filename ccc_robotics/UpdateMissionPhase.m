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
            % way point navigation 
            if (all((uvms.totalError) < 0.4)==1)
                disp('change in phase****************way -point navigation*************************');
                mission.phase = 2;
                mission.ea.poc = zeros(6,6);
                mission.ea.mac = zeros(6,6);
                mission.ea.at  = eye(3);
                mission.ea.la  = zeros(6,6);  
            end
        case 2
            % alignment 
%             disp('UpdateMissionPhase: check the uvms.xdot.at');
%             disp(uvms.xdot.at);
            disp('in mission phase 2')
            if(all((uvms.xdot.at) < 0.00002) == 1)
                disp('change in phase**************** alignment change *************************');
                mission.phase = 3;
                mission.ea.poc = zeros(6,6);
                mission.ea.mac = zeros(6,6);
                mission.ea.at  = zeros(3,3);  
                mission.ea.la  = eye(6);
            end
        case 3
            % landing 
            if( uvms.mac.wdispf < 0.2)
                mission.ea.la  = zeros(6,6); 
                mission.phase = 0;
            end
           
        case 4 
            
        
     
    end
    
end

