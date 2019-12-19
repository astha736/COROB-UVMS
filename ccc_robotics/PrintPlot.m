function [ ] = PrintPlot( plt )

disp('PrintPlot here 1');

% some predefined plots
% you can add your own

% figure('Name','Vehicle-Manipulator');
% subplot(2,1,1);
% hplot = plot(plt.t, plt.q);
% set(hplot, 'LineWidth', 1);
% legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
% subplot(2,1,2);
% hplot = plot(plt.t, plt.q_dot);
% set(hplot, 'LineWidth', 1);
% legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');
% 
figure('Name','Activation-joints');
hplot = plot(plt.t, plt.a(1:7,:));
set(hplot, 'LineWidth', 2);
legend('Ajl_1','Ajl_2','Ajl_3','Ajl_4','Ajl_5','Ajl_6','Ajl_7');

figure('Name','Activation-other');
hplot = plot(plt.t, plt.a(8:10,:));
set(hplot, 'LineWidth', 2);
legend('Amu', 'Aha', 'Amac');
    
figure('Name','Vehicle-Base');
% Vehicle-Base 1,1
subplot(2,2,1);
hplot = plot(plt.t, plt.p(1:3,:));
set(hplot, 'LineWidth', 1);
legend('x','y','z');
% Vehicle-Base 1,2
subplot(2,2,2);
hplot = plot(plt.t, plt.p(4:6,:));
set(hplot, 'LineWidth', 1);
legend('roll','pitch','yaw');
% Vehicle-Base 2,1
subplot(2,2,3);
hplot = plot(plt.t, plt.p_dot(1:3,:));
set(hplot, 'LineWidth', 1);
legend('xdot', 'ydot','zdot');

% Vehicle-Base 2,2
subplot(2,2,4);
hplot = plot(plt.t, plt.p_dot(4:6,:));
set(hplot, 'LineWidth', 1);
legend('omega_x','omega_y','omega_z');

figure('Name','Vehicle-Base-Error');
subplot(2,1,1);
hplot = plot(plt.t,plt.totalError(1:3,:));
set(hplot, 'LineWidth', 2);
legend('lx_e','ly_e','lz_e');
subplot(2,1,2);
hplot = plot(plt.t,plt.totalError(4:6,:));
set(hplot, 'LineWidth', 2);
legend('rx_e','ry_e','rz_e');

figure('Name','Vehicle-Manipulator-Error');
subplot(2,1,1);
hplot = plot(plt.t,plt.toolFrameError(1:3,:)); % 6xn
set(hplot, 'LineWidth', 2);
legend('lx_e','ly_e','lz_e');
subplot(2,1,2);
hplot = plot(plt.t,plt.toolFrameError(4:6,:));
set(hplot, 'LineWidth', 2);
legend('rx_e','ry_e','rz_e');

figure('Name','Vehicle Frame Error');
subplot(2,1,1);
% c = [];
% c(:,1) = plt.totalError(3,:);
% c(:,2) = plt.p(3,:);
hplot = plot(plt.t,plt.totalError(3,:)); % 6xn
hold on
hplot = plot(plt.t,plt.p(3,:)); % 6xn
hold off
set(hplot, 'LineWidth', 2);
legend('v_z error','v_z pos');
subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot(3,:));
set(hplot, 'LineWidth', 1);
legend('zdot');
% 
% figure('Name','Tool Frame Error');
% hplot = plot(plt.t,plt.toolFrameError(1:3,:)); % 6xn
% set(hplot, 'LineWidth', 2);
% legend('vechicle disp from floor');
% 
% disp('PrintPlot here 2');


% figure('Name','Task 1.2 disp from sea floor');
% hplot = plot(plt.t,plt.distanceFloor); % 6xn
% set(hplot, 'LineWidth', 2);
% legend('vechicle disp from floor');
% 
% disp('PrintPlot here 2');


end

