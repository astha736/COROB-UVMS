function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% example: manipulability
% if mu < 0.02, A = 1;
% if mu > 0.05, A = 0;
% in between, there is a smooth behavior.
uvms.A.mu = mission.ea.mu.*DecreasingBellShapedFunction(0.02, 0.05, 0, 1, uvms.mu);

% phi: misalignment vector between the horizontal plane and the
% longitudinal axis of the vehicle
% if norm(phi) > 0.1,   A = 1;
% if norm(phi) < 0.025, A = 0;
% in between, there is a smooth behavior.
uvms.A.ha = mission.ea.ha.*IncreasingBellShapedFunction(0.025, 0.1, 0, 1, norm(uvms.phi));

% arm tool position control
% always active
uvms.A.t = mission.ea.t.*eye(6);

% Position-control for base
% always active
uvms.A.posc = eye(6);
uvms.A.posc = mission.ea.poc .* uvms.A.posc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% we need to change the activation uvms.mac.wdispf
uvms.A.mac(3,3) = DecreasingBellShapedFunction(uvms.mac.thresh ,uvms.mac.thresh + uvms.mac.buff, 0, 1, uvms.mac.wdispf);
uvms.A.mac = mission.ea.mac.*uvms.A.mac;
uvms.A.la(3,3) = 1; %IncreasingBellShapedFunction(0.00, 0.1, 0, 1, uvms.mac.wdispf); 
uvms.A.la = mission.ea.la.*uvms.A.la;
uvms.A.at = eye(3);%
uvms.A.at = mission.ea.at.*uvms.A.at;

% non-reactive task
uvms.A.nr = eye(6);
uvms.A.nr  = mission.ea.nr.*uvms.A.nr;

% joint-limit task 
buff = 0.5;
% A_lmin = eye(7);
for i=1:7
    uvms.A.jl(i,i) = DecreasingBellShapedFunction(uvms.jlmin(i), uvms.jlmin(i)+buff, 0, 1, uvms.q(i)); + IncreasingBellShapedFunction(uvms.jlmax(i) - buff, uvms.jlmax(i), 0, 1,uvms.q(i));
end

uvms.A.jl = mission.ea.jl.*uvms.A.jl;
