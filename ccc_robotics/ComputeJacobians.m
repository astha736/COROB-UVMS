function [uvms] = ComputeJacobians(uvms,mission)
% compute the relevant Jacobians here
% joint limits
% manipulability
% tool-frame position control
% vehicle-frame position control
% horizontal attitude 
% minimum altitude
% preferred arm posture ( [-0.0031 1.2586 0.0128 -1.2460] )
%
% remember: the control vector is:
% [q_dot; p_dot] 
% [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
%
% therefore all task jacobians should be of dimensions
% m x 13
% where m is the row dimension of the task, and of its reference rate

% compute manipulability Jacobian
[Jmu_a, uvms.mu] = ComputeManipulability(uvms.bJe, uvms.djdq);
uvms.Jmu = [Jmu_a zeros(1,6)];

% computation for tool-frame Jacobian
% [omegax_t omegay_t omegaz_t xdot_t ydot_t zdot_t] = Jt ydot
% [angular velocities; linear velocities]
%
% Ste is the rigid body transformation from vehicle-frame to end-effector
% frame projected on <v>
uvms.Ste = [eye(3) zeros(3);  -skew(uvms.vTe(1:3,1:3)*uvms.eTt(1:3,4)) eye(3)];
% uvms.bJe contains the arm end-effector Jacobian (6x7) wrt arm base
% top three rows are angular velocities, bottom three linear velocities
uvms.Jt_a  = uvms.Ste * [uvms.vTb(1:3,1:3) zeros(3,3); zeros(3,3) uvms.vTb(1:3,1:3)] * uvms.bJe;
% vehicle contribution is simply a rigid body transformation from vehicle
% frame to tool frame. Notice that linear and angular velocities are
% swapped due to the different definitions of the task and control
% variables
uvms.Jt_v = [zeros(3) eye(3); eye(3) -skew(uvms.vTt(1:3,4))];
% juxtapose the two Jacobians to obtain the global one
uvms.Jt = [uvms.Jt_a uvms.Jt_v];

% horizontal attitude Jacobian
kv   = [0 0 1]';
w_kw = [0 0 1]';
v_kw = (uvms.wTv(1:3,1:3))' * w_kw;
uvms.phi   = ReducedVersorLemma(v_kw, kv);
if (norm(uvms.phi) > 0)
    nphi = uvms.phi/norm(uvms.phi);
else
    nphi = [0 0 0]';
end
uvms.Jha = [zeros(1,7) nphi'*[zeros(3) eye(3)]];

% Position-control
wRv = uvms.wTv(1:3,1:3); % world_Rotation_vehicle
uvms.Jposc = [zeros(3,7) wRv zeros(3,3);zeros(3,7) zeros(3,3) wRv];

% Minimum Altitude Control from sea floor 
uvms.Jmac = [zeros(3,7) wRv zeros(3,3);zeros(3,7) zeros(3,3) zeros(3,3)];

% landing objective
uvms.Jla = [zeros(3,7) wRv zeros(3,3);zeros(3,7) zeros(3,3) zeros(3,3)];


% misalignemnt objective
k = [0,0,1]';
horz_proj = eye(3) - k*k'; % projection in horizontal plane (for a vector expressed in world frame)

distance_rock = uvms.p(1:3,1) - mission.rock_center; % uvms.vTw(1:3,1:3)*horz_proj*distance_rock;
uvms.dist_rock_proj = uvms.vTw(1:3,1:3)*horz_proj*distance_rock;

v_HorProj_v = uvms.vTw(1:3,1:3)*horz_proj*uvms.wTv(1:3,1:3);% vRw * hori_plane * wRv  %*uvms.p_dot(1:3);
norm_r = norm(uvms.dist_rock_proj);
if(norm_r == 0)
    Jat_1 = zeros(3,3);
else
    Jat_1 = (-1*skew(uvms.dist_rock_proj)* v_HorProj_v)/(norm_r^2); % -1x(r_skew x Vp_v)*(1/norm(r))
end 

Jat_2 = -1*eye(3); 
uvms.Jat = [zeros(3,7),Jat_1,Jat_2];

%%%%%%%%%%%%%%%%%%%%%%% non-reactive task %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
uvms.Jnr = uvms.Jposc; 



end