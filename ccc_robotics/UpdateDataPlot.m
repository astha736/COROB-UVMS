function [ plt ] = UpdateDataPlot( plt, uvms, t, loop )
disp('Update Data Plotq 1');
% this function samples the variables contained in the structure uvms
% and saves them in arrays inside the struct plt
% this allows to have the time history of the data for later plots

% you can add whatever sampling you need to do additional plots
% plots are done in the PrintPlot.m script

plt.t(loop) = t;

plt.toolPos(:, loop) = uvms.wTt(1:3,4);

plt.q(:, loop) = uvms.q;
plt.q_dot(:, loop) = uvms.q_dot;
plt.q_ddot(:, loop) = uvms.q_ddot;

plt.p(:, loop) = uvms.p;
plt.p_dot(:, loop) = uvms.p_dot;
plt.p_ddot(:, loop) = uvms.p_ddot;

%plt.xdot_jl(:, loop) = uvms.xdot.jl;
plt.xdot_mu(:, loop) = uvms.xdot.mu;
plt.xdot_t(:, loop) =  blkdiag(uvms.wTv(1:3,1:3), uvms.wTv(1:3,1:3))*uvms.xdot.t;

plt.a(1:7, loop) = diag(uvms.A.jl);
plt.a(8, loop) = uvms.A.mu;
plt.a(9, loop) = uvms.A.ha(1,1);
plt.a(10,loop) = uvms.A.mac(3,3);
plt.a(11,loop) = uvms.A.la(3,3);

plt.a_posc_ha(1:6, loop) = diag(uvms.A.posc);
plt.a_at(1:3,loop) = diag(uvms.A.at);
plt.a_mp(1:4,loop) = diag(uvms.A.mp);


plt.toolFrameError(:, loop) = uvms.toolFrameError;
plt.totalError(:, loop) = uvms.totalError;

plt.toolx(:,loop) = uvms.wTt(1,4);
plt.tooly(:,loop) = uvms.wTt(2,4);

% distance of the vehicle from the sea floor (z projection(w) of laser data)
plt.distanceFloor(:,loop) = uvms.mac.wdispf;
disp('Update Data Plotq 1');
end