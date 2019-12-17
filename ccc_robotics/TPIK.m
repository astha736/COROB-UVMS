function [Qp, rhop] = TPIK(uvms,subtask)
% main kinematic algorithm initialization
% rhop order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
rhop = zeros(13,1);
Qp = eye(13); 
% add all the other tasks here!
% the sequence of iCAT_task calls defines the priority

if(subtask == 2)
    uvms.xdot.nr = uvms.p_dot;
    uvms.Jt = [uvms.Jt_a zeros(6,6)];
end

[Qp, rhop] = iCAT_task(uvms.A.nr,   uvms.Jnr,   Qp, rhop, uvms.xdot.nr, 0.0001,   0.01, 10);
[Qp, rhop] = iCAT_task(uvms.A.ha,   uvms.Jha,   Qp, rhop, uvms.xdot.ha, 0.0001,   0.01, 10);
[Qp, rhop] = iCAT_task(uvms.A.mu,   uvms.Jmu,   Qp, rhop, uvms.xdot.mu, 0.000001, 0.0001, 10);
% this robot does not have laser sensor and hence, the if statement in
% Activation will set uvms.A.mac = zeros(6,6)
[Qp, rhop] = iCAT_task(uvms.A.mac,   uvms.Jmac,   Qp, rhop, uvms.xdot.mac, 0.0001,   0.01, 10);

% Position-Control task
[Qp, rhop] = iCAT_task(uvms.A.posc, uvms.Jposc, Qp, rhop, uvms.xdot.posc, 0.0001, 0.01, 10);

[Qp, rhop] = iCAT_task(uvms.A.t,    uvms.Jt,    Qp, rhop, uvms.xdot.t,  0.0001,   0.01, 10);

[Qp, rhop] = iCAT_task(uvms.A.mp,   uvms.Jmp,   Qp, rhop, uvms.xdot.mp, 0.0001,   0.01, 10);
[Qp, rhop] = iCAT_task(eye(13),     eye(13),    Qp, rhop, zeros(13,1),  0.0001,   0.01, 10);    % this task should be the last one


end

