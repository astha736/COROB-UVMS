function [mission] = InitMission()
mission.phase = 1; % way-point navigation,  position control
mission.phase_time = 0;

mission.ea.mu = 1;
mission.ea.t = zeros(6,6);
mission.ea.ha = eye(6);
mission.ea.poc = eye(6);
mission.ea.mac = eye(6,6);
mission.ea.la = zeros(6,6);

end

