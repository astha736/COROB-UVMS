 function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for manipulability
uvms.xdot.mu = 0.1 * (0.12 - uvms.mu);

% reference for tool-frame position control task
[ang_tool, lin_tool] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.2 * [ang_tool; lin_tool];
uvms.toolFrameError = [lin_tool;ang_tool];


% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);

% reference for horizontal attitude
uvms.xdot.ha = -0.1 * norm(uvms.phi); 

% position-control reference for vehicle-frame
% doubt do we need to saturate them? - or do we add a task?
[ang_posc, lin_posc] = CartError(uvms.wTgpos, uvms.wTv);
uvms.xdot.posc = 0.2*[lin_posc; ang_posc]; %6x1 vector
uvms.totalError = [lin_posc;ang_posc]; %6x1


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
norm_dist = norm(uvms.dist_rock_proj); % the distance vector is in the vehicle frame 


rho_dir =cross([1,0,0]',uvms.dist_rock_proj/norm_dist) ; % axb of the unit vectors 
uvms.theta = asin(norm(rho_dir)); % get the theta value 
rho = rho_dir*uvms.theta ; % rho = n*theta
uvms.xdot.at = rho*5;

%%%%%%%%%%%%%%%%%%%%%%% non-reactive task %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% [ang_posc, lin_posc] = CartError(uvms.landing_pos, uvms.wTv);
% uvms.xdot.nr = 0.2*[lin_posc; ang_posc]; %6x1 vector

uvms.xdot.nr = [0,0,0,0,0,0]';

%%%%%%%%%%%%%%%%%%%%%%% Joint limit task %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
uvms.xdot.jl = [0 0 0 0 0 0 0]';

end


