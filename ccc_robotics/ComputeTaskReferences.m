function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for manipulability
uvms.xdot.mu = 0.1 * (0.12 - uvms.mu);

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.2 * [ang; lin];
uvms.toolFrameError = [lin;ang];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);

% reference for horizontal attitude
uvms.xdot.ha = -0.1 * norm(uvms.phi); 

% position-control reference for vehicle-frame
% doubt do we need to saturate them? - or do we add a task?
[ang, lin] = CartError(uvms.wTgpos, uvms.wTv);
uvms.xdot.posc = 0.2*[lin; ang]; %6x1 vector
uvms.totalError = [lin;ang]; %6x1

% call the sensor data to check the actual distance of the base from
% the sea floor
% fill this up correctly
% check if the projections are correct 
temp = uvms.wTv*[0,0,uvms.sensorDistance,0]';
uvms.mac.wdispf = temp(3);
mac_velocity_upwards = 0.2*(uvms.mac.thresh + uvms.mac.buff - uvms.mac.wdispf);
uvms.xdot.mac = [0, 0,mac_velocity_upwards,0,0,0 ]';