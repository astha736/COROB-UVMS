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
uvms.linErrorVehicle = zeros(3,1);
uvms.angErrorVehicle = zeros(3,1);
[uvms.angErrorVehicle, uvms.linErrorVehicle] = CartError(uvms.wTgpos, uvms.wTv);
uvms.xdot.posc = 0.2*[uvms.linErrorVehicle; uvms.angErrorVehicle]; %6x1 vector
uvms.totalError = [uvms.linErrorVehicle;uvms.angErrorVehicle]; %6x1

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
lin_v_proj = uvms.vTw(1:3,1:3)*horz_proj*uvms.linErrorVehicle;
norm_r = norm(lin_v_proj);
rho_dir =cross([1,0,0]',lin_v_proj/norm_r) ; % axb of the unit vectors 
theta = asin(norm(rho_dir)); % get the theta value 
rho = rho_dir*theta; % rho = n*theta
uvms.xdot.at = rho*0.2;






