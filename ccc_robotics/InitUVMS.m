function [uvms] = InitUVMS(robotname)

% uvms.vTb
% transformation matrix betwene the arm base wrt vehicle frame
% expresses how the base of the arm is attached to the vehicle
% do NOT change, since it must be coherent with the visualization tool
if (strcmp(robotname, 'DexROV'))    
    % do NOT change
    uvms.vTb = [rotation(pi, 0, pi) [0.167 0 -0.43]'; 0 0 0 1]; 
else
    if (strcmp(robotname, 'Robust'))
        % do NOT change
        uvms.vTb = [rotation(0, 0, pi) [0.85 0 -0.42]'; 0 0 0 1];
    end
end

uvms.q_dot = [0 0 0 0 0 0 0]';
uvms.q_ddot = [0 0 0 0 0 0 0]';
uvms.p_dot = [0 0 0 0 0 0]';
uvms.p_ddot = [0 0 0 0 0 0]';

% joint limits corresponding to the actual MARIS arm configuration
uvms.jlmin  = [-2.9;-1.6;-2.9;-2.95;-2.9;-1.65;-2.8];
uvms.jlmax  = [2.9;1.65;2.9;0.01;2.9;1.25;2.8];

% to be compute at each time step
uvms.wTv = eye(4,4);
uvms.wTt = eye(4,4);
uvms.vTw = eye(4,4);
uvms.vTe = eye(4,4);
uvms.vTt = eye(4,4);
uvms.vTg = eye(4,4);
uvms.Ste = eye(6,6);
uvms.bTe = eye(4,4);
uvms.bJe = eye(6,7);
uvms.djdq = zeros(6,7,7);
uvms.mu  = 0;
uvms.phi = zeros(3,1);
uvms.psi = zeros(3,1);
uvms.xi = zeros(3,1);
uvms.virtualFrameVelocity = zeros(6,1);
uvms.sensorDistance = 0;

uvms.wTgpos = eye(4); % position-control
uvms.wTipos = eye(4); % position-control


uvms.Jjl = [];
uvms.Jmu = [];
uvms.Jcc = [];
uvms.Jha = [];
uvms.Jt_a = [];
uvms.Jt_v = [];
uvms.Jt = [];
uvms.Jc = [];
uvms.Jca = [];
uvms.Jposc = [];    % position-control Jacobian
uvms.Jmac = [];     % minimum altitude control 
uvms.Jla = [];      % landing objective
uvms.Jat = [];      % misalignmnet objective
uvms.Jnr = [];      % non-reactive task 
uvms.Jmp = [];      % manipulator position task 5.1
    
uvms.xdot.jl = [];
uvms.xdot.mu = [];
uvms.xdot.cc = [];
uvms.xdot.ha = [];
uvms.xdot.t = [];
uvms.xdot.c = [];
uvms.xdot.ca = [];
uvms.xdot.posc = [];    % position-control Refrence velocity
uvms.xdot.mac = [];     % minimum altitude control 
uvms.xdot.la = [];      % landing objective
uvms.xdot.at = [];      % misalignmnet objective 
uvms.xdot.nr = [];      % non-reactive task 
uvms.xdot.mp = [];      % manipulator position task 5.1

uvms.A.jl = zeros(7,7);
uvms.A.mu = 0;
uvms.A.cc = zeros(1,1);
uvms.A.ha = zeros(1,1);
uvms.A.t = zeros(6,6);
uvms.A.c = [];
uvms.A.ca = zeros(3,3);
uvms.A.posc = zeros(6,6);% position-control Activation
uvms.A.mac = zeros(6,6); % minimum altitude control 
uvms.A.la = zeros(6,6);  % landing objective
uvms.A.at = zeros(3,3);  % misalignmnet objective 
uvms.A.nr = zeros(6,6);  % non-reactive task
uvms.A.mp = eye(4,4);    % manipulator position task 5.1

uvms.toolFrameError = zeros(6,1);
uvms.totalError = zeros(6,1);
% uvms.linErrorVehicle = zeros(3,1);
% uvms.angErrorVehicle = zeros(3,1);
uvms.dist_rock_proj = zeros(3,1);
uvms.theta = 1;

uvms.mac.thresh = 1;
uvms.mac.buff = 1;
uvms.mac.wdispf = 0;

% uvms.landing_pos = eye(4); % to save the position where we landed
end

