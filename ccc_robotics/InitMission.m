function [mission] = InitMission(robotname)

mission.robot = 0;
mission.phase = 0; % way-point navigation,  position control
mission.phase_time = 0;

if (strcmp(robotname, 'DexROV'))    
    mission.robot = 1;
end

% activation for robust robot 
mission.ea.mu = 0;
mission.ea.ha = 0;
mission.ea.t = zeros(6,6);

% activation for  robust robot 
mission.ea.poc = zeros(6,6);
mission.ea.mac = zeros(3,3);
mission.ea.la = zeros(3,3);
mission.ea.at = zeros(3,3);
mission.ea.nr = zeros(6,6);
mission.ea.jl = zeros(7,7);
mission.ea.mp = zeros(4,4);

% variables required for a mission 
% set by the main file of robust
mission.rock_center = [0,0,0]';

mission.preffered_shape = [0 0 0 0]';

if mission.robot == 0
    mission.rock_center  = [12.2025   37.3748  -39.8860]'; % in world frame coordinates
else
    mission.preffered_shape = [-0.0031 1.2586 0.0128 -1.2460]';
end

end

