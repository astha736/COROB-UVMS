function [uvms, mission] = UpdateMissionPhase(uvms, mission)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % % % % activation for robust robot 
% % % % % mission.ea.mu = 0;
% % % % % mission.ea.ha = 0;
% % % % % mission.ea.t = zeros(6,6);
% % % % % 
% % % % % % activation for  robust robot 
% % % % % mission.ea.poc = zeros(6,6);
% % % % % mission.ea.mac = zeros(3,3);
% % % % % mission.ea.la = zeros(3,3);
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
                mission.ea.ha = 1;
                mission.ea.mu = 1; 
                
                 mission.ea.jl = eye(7);
                
                mission.ea.mac = eye(3);
                mission.ea.poc = eye(6);
                
                mission.ea.la  = zeros(3,3); 
                mission.ea.at  = zeros(3,3);
                
                
                mission.ea.t = zeros(6,6);
                mission.ea.nr = zeros(6,6);
                mission.ea.mp = zeros(4,4);
                
        case 1  
%             way point navigation 
            if ((norm(uvms.totalError(1:3)) < 0.4) && (norm(uvms.totalError(3:6)) < 0.2)) %%%%% ? maybe change this to norms
                % set the external activation for case 2: alignment 
                mission.phase = 2; % alignment task
                
                % in phase 2 in next loop for alignment task 
                mission.ea.ha = 1;
                mission.ea.mu = 1; 
                              
                mission.ea.mac = eye(3);
                mission.ea.at  = eye(3);
                
                mission.ea.jl = eye(7);
                
                mission.ea.la  = zeros(3,3); 
                mission.ea.poc = zeros(6,6);
                
                mission.ea.t = zeros(6,6);
                mission.ea.nr = zeros(6,6);
                mission.ea.mp = zeros(4,4);
            end
         
        case 2
            
            if((uvms.theta < 0.1) == 1) % 0.05 origibnal value 
                
                mission.phase = 3; % landing task 
                
                % landing task 
                mission.ea.ha = 1;
                mission.ea.mu = 1; 
                
                mission.ea.la  = eye(3); 
                mission.ea.at  = eye(3);
                mission.ea.jl = eye(7);
                
                mission.ea.mac = zeros(3,3);
                

                mission.ea.poc = zeros(6,6);
                mission.ea.t = zeros(6,6);
                mission.ea.nr = zeros(6,6);
                mission.ea.mp = zeros(4,4);
            end
    
        case 3
            % landing task
            if( uvms.mac.wdispf < 0.1) %  0.05 original value the world projection_z of the sensor from floor  landing 
                mission.phase = 4;  % tool frame control
                
                % tool frame control
                mission.ea.ha = 1;
                mission.ea.mu = 1; % ???????? maybe this should be zero ?
                mission.ea.t = eye(6);
                mission.ea.nr = eye(6);
                mission.ea.jl = eye(7);
                
                
                
                mission.ea.mac = zeros(3,3);
                mission.ea.la  = zeros(3,3);  
                mission.ea.at  = zeros(3,3);
                mission.ea.poc = zeros(6,6);
                mission.ea.mp = zeros(4,4);
                
%                 uvms.landing_pos = uvms.wTv;           
            end
           
        case 4
%             disp(' tool frame control *************************');
            disp('uvms.A.jl');
            disp(diag(uvms.A.jl));
            
            disp('uvms.q');
            disp(uvms.q);

     
    end
else
%     disp('DexROV');
%     disp('****************************************************************')
% here we write the logic to change phase 
    switch mission.phase
        case 0  %default values 
%                 disp('case 0 UpdateMissionPhaseDex ');
                % case of safe-way point navigation 
                mission.phase = 1; 
                
                % init for safe-way point navigation 
                mission.ea.ha = 1;
                mission.ea.mu = 1;
                mission.ea.jl = eye(7);
                 
                mission.ea.mp = eye(4);
                mission.ea.poc = eye(6);
                
                mission.ea.t =  zeros(6,6);
                mission.ea.nr = zeros(6,6);
               
                
        case 1 % case of safe-way point navigation 
            if (norm(uvms.totalError) < 0.4)
                %for case 2 --- get the robot arm at the right pos
                mission.phase = 2;
                
                mission.ea.ha = 1;
                mission.ea.mu = 1; % ?????????? set to 0?
                mission.ea.t = eye(6);
                mission.ea.mp = eye(4);
                mission.ea.nr = eye(6);
                mission.ea.jl = eye(7);
                
%                 mission.ea.mac = zeros(3,3);;
                
               
                mission.ea.poc = zeros(6,6);
            end
        case 2
              
              
        case 3          
        case 4
    end
end
end

