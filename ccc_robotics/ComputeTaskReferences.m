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
