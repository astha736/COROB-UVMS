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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%% Position-Control for base %%%%%%%%%%%%%%%%%%%%%%%
% position-control reference for vehicle-frame
% doubt do we need to saturate them? - or do we add a task?
[ang_posc, lin_posc] = CartError(uvms.wTgpos, uvms.wTv);
uvms.xdot.posc = [0.2*lin_posc; 2*ang_posc]; %6x1 vector
uvms.totalError = [lin_posc;ang_posc]; %6x1

%%%%%%%%%%%%%%%%%%%%%%%% Minimum Altitude Control  %%%%%%%%%%%%%%%%%%%%%%%
% call the sensor data to check the actual distance of the base from
% the sea floor
% fill this up correctly
% check if the projections are correct 
temp = uvms.wTv*[0,0,uvms.sensorDistance,0]';
uvms.mac.wdispf = temp(3);
mac_velocity_upwards = 0.2*(uvms.mac.thresh + uvms.mac.buff - uvms.mac.wdispf);
uvms.xdot.mac = [0, 0,mac_velocity_upwards]';

%%%%%%%%%%%%%%%%%%%%%%%% Landing Objective %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
la_velocity_downwards = -0.4*uvms.mac.wdispf;
uvms.xdot.la = [0, 0,la_velocity_downwards ]';

%%%%%%%%%%%%%%%%%%%%%%%% Misalignmnet Objective  %%%%%%%%%%%%%%%%%%%%%%%%%%
% the distance vector
% is in the vehicle frame 
norm_dist = norm(uvms.dist_rock_proj);  

% axb of the unit vectors = neta*sin(theta)
rho_dir =cross([1,0,0]',uvms.dist_rock_proj/norm_dist); 
% get the theta value 
uvms.theta = asin(norm(rho_dir)); 
% rho = n*theta
rho = rho_dir*uvms.theta ; 
uvms.xdot.at = rho*5; % have to chnage this to suitable value 

%%%%%%%%%%%%%%%%%%%%%%%%% Non-Reactive task %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
uvms.xdot.nr = [0,0,0,0,0,0]';

%%%%%%%%%%%%%%%%%%%%%%% Joint limit task %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
uvms.xdot.jl = [0 0 0 0 0 0 0]';
gain = 0.8;
for i=1:7
%     uvms.A.jl(i,i) = DecreasingBellShapedFunction(uvms.jlmin(i), uvms.jlmin(i)+buff, 0, 1, uvms.q(i)); + IncreasingBellShapedFunction(uvms.jlmax(i) - buff, uvms.jlmax(i), 0, 1,uvms.q(i));
    if(uvms.A.jl_flag(i) ==1)
        uvms.xdot.jl(i) = gain*(uvms.jlmin(i) + uvms.buff.jl  - uvms.q(i));
    elseif(uvms.A.jl_flag(i) == 2)
        uvms.xdot.jl(i) = gain*(uvms.jlmax(i) - uvms.buff.jl  - uvms.q(i));
    end
    
end

%%%%%%%%%%%%%%%%%%%%%% Manipulator Position task 5.1  %%%%%%%%%%%%%%%%%%%%%
uvms.xdot.mp = 0.2*(mission.preffered_shape - uvms.q(1:4,1));

end


