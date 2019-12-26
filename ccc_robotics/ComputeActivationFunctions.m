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
% uvms.A.ha = mission.ea.ha.*IncreasingBellShapedFunction(0.025, 0.1, 0, 1, norm(uvms.phi));
uvms.A.ha = mission.ea.ha.*IncreasingBellShapedFunction(0.025, 0.1, 0, 1, norm(uvms.phi));

% arm tool position control
% always active
uvms.A.t = mission.ea.t.*eye(6);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%% Position-Control for base %%%%%%%%%%%%%%%%%%%%%%%
% always active
uvms.A.posc = eye(6);
uvms.A.posc = mission.ea.poc .* uvms.A.posc;

%%%%%%%%%%%%%%%%%%%%%%%% Minimum Altitude Control  %%%%%%%%%%%%%%%%%%%%%%%
% always active
% we need to change the activation uvms.mac.wdispf
if( uvms.mac.wdispf == 0)
    uvms.A.mac(3,3) = 0;
else
    uvms.A.mac(3,3) = DecreasingBellShapedFunction(uvms.mac.thresh ,uvms.mac.thresh + uvms.mac.buff, 0, 1, uvms.mac.wdispf);
end
uvms.A.mac = mission.ea.mac.*uvms.A.mac;

%%%%%%%%%%%%%%%%%%%%%%%% Landing Objective %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%IncreasingBellShapedFunction(0.00, 0.1, 0, 1, uvms.mac.wdispf); 
uvms.A.la(3,3) = 1; 
uvms.A.la = mission.ea.la.*uvms.A.la;

%%%%%%%%%%%%%%%%%%%%%%%% Misalignmnet Objective  %%%%%%%%%%%%%%%%%%%%%%%%%%
uvms.A.at = eye(3);
uvms.A.at = mission.ea.at.*uvms.A.at;

%%%%%%%%%%%%%%%%%%%%%%%%% Non-Reactive task %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
uvms.A.nr = eye(6);
uvms.A.nr  = mission.ea.nr.*uvms.A.nr;

%%%%%%%%%%%%%%%%%%%%%%%%%% Joint-Limit task %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% uvms.buff.jl = 0.5;
% A_lmin = eye(7);     
for i=1:7
    
    if( uvms.q(i) < (uvms.jlmin(i)+ uvms.buff.jl) )
        uvms.A.jl_flag(i) = 1;
        uvms.A.jl(i,i) = DecreasingBellShapedFunction(uvms.jlmin(i), uvms.jlmin(i)+uvms.buff.jl , 0, 1, uvms.q(i));
    elseif( uvms.q(i) > (uvms.jlmax(i) - uvms.buff.jl) )
        uvms.A.jl_flag(i) = 2;
        uvms.A.jl(i,i) =  IncreasingBellShapedFunction(uvms.jlmax(i) - uvms.buff.jl , uvms.jlmax(i), 0, 1,uvms.q(i));
    else
        uvms.A.jl_flag(i) = 0;
    end
    
end

%         disp('uvms.A.jl -- before ea');
%         disp(uvms.A.jl);
uvms.A.jl = mission.ea.jl.*uvms.A.jl;

%%%%%%%%%%%%%%%%%%%%%% Manipulator Position task 5.1  %%%%%%%%%%%%%%%%%%%%%
uvms.A.mp = eye(4);
uvms.A.mp = mission.ea.mp.*uvms.A.mp;




