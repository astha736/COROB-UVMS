function [uvms, mission] = UpdateMissionPhase(uvms, mission)

% mission.phase = 1; % way-point navigation,  position control
% mission.phase_time = 0;

% mission.ea.mu = 1;
% mission.ea.t = zeros(6,6);
% mission.ea.ha = 1;
% mission.ea.poc = zeros(6,6);
% mission.ea.mac = eye(6,6);
% mission.ea.la = zeros(6,6);
% mission.ea.at = zeros(3,3);
% mission.ea.nr = zeros(6,6);

if (all(uvms.totalError) == 0)
    return
end 

% here we write the logic to change phase 
    switch mission.phase
        case 0
            % this is indication that everything is done
                mission.phase = 1;
                
                mission.ea.mac = eye(6);
                mission.ea.ha = 1;
                mission.ea.la  = zeros(6,6); 
                mission.ea.at  = zeros(3,3);
                mission.ea.poc = eye(6);
                mission.ea.mu = 1; 
                mission.ea.t = zeros(6,6);
                mission.ea.nr = zeros(6,6);
                
        case 1  
            % way point navigation 
            if (all((uvms.totalError) < 0.4)==1)
                disp('change in phase****************way -point navigation*************************');
                % set the external activation for case 2: alignment 
                mission.phase = 2;
                mission.ea.mac = eye(6);
                mission.ea.ha = 1;
                mission.ea.la  = zeros(6,6); 
                mission.ea.at  = eye(3);
                mission.ea.poc = zeros(6,6);
                mission.ea.mu = 1; 
                mission.ea.t = zeros(6,6);
                mission.ea.nr = zeros(6,6);
            end
         
        case 2
            % alignment 
            disp('in mission phase 2')
%             cross_disp_veh = cross([1,0,0]',uvms.dist_rock_proj); % axb which we use to get the direction of rho vector
            if((uvms.theta < 0.05) == 1)
                disp('change in phase**************** alignment change *************************');
                mission.phase = 3;
                mission.ea.mac = zeros(6,6);
                mission.ea.ha = 1;
                mission.ea.la  = eye(6); 
                mission.ea.at  = eye(3);
                mission.ea.poc = zeros(6,6);
                mission.ea.mu = 1; 
                mission.ea.t = zeros(6,6);
                mission.ea.nr = zeros(6,6);
            end
        case 3
            % landing 
            if( uvms.mac.wdispf < 0.05) % the world projection_z of the sensor from floor  landing 
                disp('change in phase**************** alignment change *************************');
                mission.phase = 4;  
                mission.ea.mac = zeros(6,6);
                mission.ea.ha = 1;
                mission.ea.la  = zeros(6,6);  
                mission.ea.at  = zeros(3,3);
                mission.ea.poc = zeros(6,6);
                mission.ea.mu = 0; 
                mission.ea.t = eye(6);
                mission.ea.nr = eye(6);
                
                
            end
%            
        case 4
            disp(' tool frame control *************************');
            
            
        
     
    end
    
end

