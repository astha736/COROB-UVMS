function [mission] = InitMission()
mission.phase = 0; % way-point navigation,  position control
mission.phase_time = 0;

mission.ea.mu = 1;
mission.ea.t = zeros(6,6);
mission.ea.ha = 1;
mission.ea.poc = zeros(6,6);
mission.ea.mac = eye(6,6);
mission.ea.la = zeros(6,6);
mission.ea.at = zeros(3,3);
mission.ea.nr = zeros(6,6);
mission.ea.jl = zeros(7,7);

% variables required for a mission 
mission.rock_center = [0,0,0]';

end

