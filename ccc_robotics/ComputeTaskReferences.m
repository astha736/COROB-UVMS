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
[ang_v, lin_v] = CartError(uvms.wTgpos, uvms.wTv);
uvms.xdot.posc = 0.2*[lin_v; ang_v]; %6x1 vector
uvms.totalError = [lin_v;ang_v]; %6x1

% call the sensor data to check the actual distance of the base from
% the sea floor
% fill this up correctly
% check if the projections are correct 
temp = uvms.wTv*[0,0,uvms.sensorDistance,0]';
uvms.mac.wdispf = temp(3);
mac_velocity_upwards = 0.2*(uvms.mac.thresh + uvms.mac.buff - uvms.mac.wdispf);
uvms.xdot.mac = [0, 0,mac_velocity_upwards,0,0,0 ]';

% landing objective
la_velocity_downwards = -0.2*uvms.mac.wdispf;
uvms.xdot.la = [0, 0,la_velocity_downwards,0,0,0 ]';

% misalignemnt objective 
horz_proj = [1,0,0;0,1,0;0,0,0];
lin_v_proj = uvms.vTw(1:3,1:3)*horz_proj*lin_v; % vRw * hori_plane * distance_w
vellin_v_proj = uvms.vTw(1:3,1:3)*horz_proj*uvms.wTv(1:3,1:3)*uvms.p_dot(1:3);
norm_r = norm(lin_v_proj);
norm_v = norm(vellin_v_proj);
v_w_bw =  crosss(lin_v_proj/norm_r,vellin_v_proj/norm_v)*(norm_v/norm_r);




