function [uvms, mission] = UpdateMissionPhase(uvms, mission)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % % % % activation for robust robot 
% % % % % mission.ea.mu = 0;
% % % % % mission.ea.ha = 0;
% % % % % mission.ea.t = zeros(6,6);
% % % % % 
% % % % % % activation for  robust robot 
% % % % % mission.ea.poc = zeros(6,6);
% % % % % mission.ea.mac = zeros(6,6);
% % % % % mission.ea.la = zeros(6,6);
% % % % % mission.ea.at = zeros(3,3);
% % % % % mission.ea.nr = zeros(6,6);
% % % % % mission.ea.jl = zeros(7,7);
% % % % % mission.ea.mp = zeros(4,4);
% % % % % 
% % % % % % variables required for a mission 
% % % % % % set by the main file of robust
% % % % % mission.rock_center = [0,0,0]';
% % % % % 
% % % % % mission.preffered_shape = [0 0 0 0]';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (norm(uvms.totalError) == 0) % this would mean that inital goal is not set
    
    return
end 

if mission.robot == 0
    % here we write the logic to change phase 
    disp('Robust');
    switch mission.phase
        case 0
                % this is indication that everything is done
                mission.phase = 1; % way point navigation 
                
                % way point navigation 
                % in phase 1 in next loop init 
                mission.ea.ha = 1;
                mission.ea.mu = 1; 
                
                mission.ea.mac = eye(6);
                mission.ea.poc = eye(6);
                
                mission.ea.la  = zeros(6,6); 
                mission.ea.at  = zeros(3,3);
                
                
                mission.ea.t = zeros(6,6);
                mission.ea.nr = zeros(6,6);
                mission.ea.jl = zeros(7,7);
                mission.ea.mp = zeros(4,4);
                
        case 1  
            % way point navigation 
            if (all((uvms.totalError) < 0.4)==1) %%%%% ? maybe change this to norms
                disp('change in phase****************way -point navigation*************************');
                % set the external activation for case 2: alignment 
                mission.phase = 2; % alignment 
                
                % in phase 2 in next loop for alignment task 
                mission.ea.ha = 1;
                mission.ea.mu = 1; 
                
                
                mission.ea.mac = eye(6);
                mission.ea.at  = eye(3);
                
                mission.ea.la  = zeros(6,6); 
                mission.ea.poc = zeros(6,6);
                
                mission.ea.t = zeros(6,6);
                mission.ea.nr = zeros(6,6);
                mission.ea.jl = zeros(7,7);
                mission.ea.mp = zeros(4,4);
            end
         
        case 2
            
            disp('in mission phase 2')
%             cross_disp_veh = cross([1,0,0]',uvms.dist_rock_proj); % axb which we use to get the direction of rho vector
            if((uvms.theta < 0.5) == 1) % 0.05 origibnal value 
                disp('change in phase**************** alignment change *************************');
                mission.phase = 3; % landing task 
                
                % in phase 3 in next loop for landing task
                mission.ea.ha = 1;
                mission.ea.mu = 1; 
                
                mission.ea.la  = eye(6); 
                mission.ea.at  = eye(3);
                
                mission.ea.mac = zeros(6,6);
                

                mission.ea.poc = zeros(6,6);
                mission.ea.t = zeros(6,6);
                mission.ea.nr = zeros(6,6);
                mission.ea.jl = zeros(7,7);
                mission.ea.mp = zeros(4,4);
            end
    
        case 3
            % landing task
            if( uvms.mac.wdispf < 0.1) %  0.05 original value the world projection_z of the sensor from floor  landing 
                disp('change in phase**************** alignment change *************************');
                mission.phase = 4;  % tool frame control
                
                % in phase 3 in next loop for frame control
                mission.ea.ha = 1;
                mission.ea.mu = 1; % ???????? maybe this should be zero ?
                mission.ea.t = eye(6);
                mission.ea.nr = eye(6);
                mission.ea.jl = eye(7);
                
                
                
                mission.ea.mac = zeros(6,6);
                mission.ea.la  = zeros(6,6);  
                mission.ea.at  = zeros(3,3);
                mission.ea.poc = zeros(6,6);
                mission.ea.mp = zeros(4,4);
                
%                 uvms.landing_pos = uvms.wTv;           
            end
%            
        case 4
            disp(' tool frame control *************************');
     
    end
else
    disp('DexROV')
% here we write the logic to change phase 
    switch mission.phase
%         disp('switch case');
        case 0  %default values 
                disp('case 0 UpdateMissionPhaseDex ');
                % case of safe-way point navigation 
                mission.phase = 1; 
                
                % init for safe-way point navigation 
                mission.ea.ha = 1;
                mission.ea.mu = 1; 
                 
                mission.ea.mp = eye(4);
                mission.ea.poc = eye(6);
                
                mission.ea.t = zeros(6,6);
                
        case 1 % case of safe-way point navigation 
            disp('case 1 UpdateMissionPhaseDex');
            disp('error');
            disp(uvms.totalError);
            if (all((uvms.totalError) < 0.4)==1) % ?????????? switch to norm?
                % for case 2
                mission.phase = 2;
                
                mission.ea.ha = 1;
                mission.ea.mu = 1; % ?????????? set to 0?
                mission.ea.t = eye(6);
                
                mission.ea.mp = zeros(4,4);
                mission.ea.poc = zeros(6,6);
            end
        case 2
            disp('case 2');
            disp('error');
            disp(uvms.totalError);
            
            disp('uvms.toolFrameError');
            disp(uvms.toolFrameError);
            
            disp('uvms.q');
            disp(uvms.q);
        case 3          
        case 4
    end
end
end

