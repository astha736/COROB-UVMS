function MainRobust
addpath('./simulation_scripts');
clc;
clear;
close all

% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 25;
loop = 1;
maxloops = ceil(end_time/deltat);

% this struct can be used to evolve what the UVMS has to do
mission.phase = 1;
mission.phase_time = 0;

% Rotation matrix to convert coordinates between Unity and the <w> frame
% do not change
wuRw = rotation(0,-pi/2,pi/2);
vRvu = rotation(-pi/2,0,-pi/2);

% pipe parameters
u_pipe_center = [-10.66 31.47 -1.94]'; % in unity coordinates
pipe_center = wuRw'*u_pipe_center;     % in world frame coordinates
pipe_radius = 0.3;

% rock position 
rock_center = [12.2025   37.3748  -39.8860]'; % in world frame coordinates

% UDP Connection with Unity viewer v2
uArm = udp('127.0.0.1',15000,'OutputDatagramPacketSize',28);
uVehicle = udp('127.0.0.1',15001,'OutputDatagramPacketSize',24);
fopen(uVehicle);
fopen(uArm);
uAltitude = dsp.UDPReceiver('LocalIPPort',15003,'MessageDataType','single');
uAltitude.setup();

% Preallocation
plt = InitDataPlot(maxloops);

% initialize uvms structure
uvms = InitUVMS('Robust');
% uvms.q 
% Initial joint positions. You can change these values to initialize the simulation with a 
% different starting position for the arm
uvms.q = [-0.0031 0 0.0128 -1.2460 0.0137 0.0853-pi/2 0.0137]'; 
% uvms.p
% initial position of the vehicle
% the vector contains the values in the following order
% [x y z r(rot_x) p(rot_y) y(rot_z)]
% RPY angles are applied in the following sequence
% R(rot_x, rot_y, rot_z) = Rz (rot_z) * Ry(rot_y) * Rx(rot_x)
%uvms.p = [8.5 38.5 -38   0 -0.06 0.5]'; 
uvms.p = [48.5 11.5 -33 0 0 -pi/2]';
%uvms.p = [10.5 35.5 -36 0 0  pi/2]';

% defines the goal position for the end-effector/tool position task
uvms.goalPosition = [12.2025   37.3748  -39.8860]';
uvms.wRg = rotation(0, pi, pi/2);
uvms.wTg = [uvms.wRg uvms.goalPosition; 0 0 0 1];

% position-control goal Position
% uvms.gpos = [10.5 37.5 -38 degtorad(45) degtorad(45) 0]';
%uvms.gpos = [10.5 37.5 -38 0 0 0]';
uvms.gpos = [50 12.5 -53 0 0 -pi/2]'; 
wRgpos = rotation(uvms.gpos(4),uvms.gpos(5),uvms.gpos(6));
uvms.wTgpos = [wRgpos uvms.gpos(1:3); 0 0 0 1];

% defines the tool control point
uvms.eTt = eye(4);

% added to avoid initial jump
uvms = ReceiveUdpPackets(uvms, uAltitude);

tic
for t = 0:deltat:end_time
    % update all the involved variables
    uvms = UpdateTransforms(uvms);
    uvms = ComputeJacobians(uvms);
    uvms = ComputeTaskReferences(uvms, mission);
    uvms = ComputeActivationFunctions(uvms, mission);
    
    % receive altitude information from unity
    uvms = ReceiveUdpPackets(uvms, uAltitude);
    
    disp('vehicle-velocity');
    disp(uvms.p_dot);
    
    % main kinematic algorithm initialization
    % rhop order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    rhop = zeros(13,1);
    Qp = eye(13); 
    % task (activation -> only gets activated when base is close to the floor)
    % velocity -> upward velocity prop to error between the minimun
    % (distance - actual distance )
    % jacobian will be the same as position control
    [Qp, rhop] = iCAT_task(uvms.A.mac,   uvms.Jmac,   Qp, rhop, uvms.xdot.mac, 0.0001,   0.01, 10);
    % the sequence of iCAT_task calls defines the priority
    [Qp, rhop] = iCAT_task(uvms.A.ha,   uvms.Jha,   Qp, rhop, uvms.xdot.ha, 0.0001,   0.01, 10); 
    % Position-Control task
    [Qp, rhop] = iCAT_task(uvms.A.posc, uvms.Jposc, Qp, rhop, uvms.xdot.posc, 0.0001, 0.01, 10);
    
    [Qp, rhop] = iCAT_task(uvms.A.mu,   uvms.Jmu,   Qp, rhop, uvms.xdot.mu, 0.000001, 0.0001, 10);
    %[Qp, rhop] = iCAT_task(uvms.A.t,    uvms.Jt,    Qp, rhop, uvms.xdot.t,  0.0001,   0.01, 10);
    [Qp, rhop] = iCAT_task(eye(13),     eye(13),    Qp, rhop, zeros(13,1),  0.0001,   0.01, 10);    % this task should be the last one
    
    % get the two variables for integration
    uvms.q_dot = rhop(1:7);
    uvms.p_dot = rhop(8:13);
    
    % Integration
	uvms.q = uvms.q + uvms.q_dot*deltat;
    % beware: p_dot should be projected on <v>
    uvms.p = integrate_vehicle(uvms.p, uvms.p_dot, deltat);
    
    % check if the mission phase should be changed
    [uvms, mission] = UpdateMissionPhase(uvms, mission);
    
    % send packets to Unity viewer
    SendUdpPackets(uvms,wuRw,vRvu,uArm,uVehicle);
        
    % collect data for plots
    plt = UpdateDataPlot(plt,uvms,t,loop);
    loop = loop + 1;
   
    % add debug prints here
    if (mod(t,0.1) == 0)
        t
        uvms.sensorDistance
    end

    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    SlowdownToRealtime(deltat);
end

fclose(uVehicle);
fclose(uArm);

PrintPlot(plt);

end