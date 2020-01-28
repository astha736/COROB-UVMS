function [ ] = PrintPlot( plt )

disp('PrintPlot here 1');

% some predefined plots
% you can add your own

figure('Name','Vehicle-Manipulator');
subplot(2,1,1);
plot(plt.t, plt.q, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
xlabel( 'time [s]');
ylabel('joint position [rad]')

subplot(2,1,2);
plot(plt.t, plt.q_dot,'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');
xlabel( 'time [s]');
ylabel('joint velocity [rad/s]')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Name','Activation-joints');
hplot = plot(plt.t, plt.a(1:7,:),'LineWidth', 2);
set(hplot, 'LineWidth', 2);
legend('Ajl_1','Ajl_2','Ajl_3','Ajl_4','Ajl_5','Ajl_6','Ajl_7');
xlabel( 'time [s]');
ylabel('Activation')

%%%%%%%%%%%%%%%%%%%%%%%%Activation-Vehicle%%%%%%%%%%%%%%%%%%%%
figure('Name','Activation-Vehicle');
sgtitle('Activations for Vehicle');
plot(plt.t, plt.a(8:11,:),'LineWidth', 2);
legend('Amu', 'Aha','Amac', 'Ala');
xlabel( 'time [s]');
ylabel('Activation')


%%%%%%%%%%%%%%%%%%%%% Activation-Vehicle Position Control %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure('Name','Activation-Vehicle Position Control');
sgtitle('Activations for Vehicle Position');
plot(plt.t, plt.a_posc_ha(1:6,:),'LineWidth', 2);
hold on 
plot(plt.t, plt.a(9,:),'LineWidth', 2);
legend('Aposc_1','Aposc_2','Aposc_3','Aposc_4','Aposc_5','Aposc_6', 'Aha');
xlabel( 'time [s]');
ylabel('Activations')

%%%%%%%%%%%%%%%%%%%%%%%%Vehicle-Base-Pose-Velocity%%%%%%%%%%%%%%%%%%%%
figure('Name','Vehicle-Base-Pose-Velocity');
% Vehicle-Base 1,1
subplot(2,2,1);
plot(plt.t, plt.p(1:3,:),'LineWidth', 2);
xlabel( 'time [s]');
ylabel('position [m]')
legend('x','y','z');
title('Position');
% Vehicle-Base 1,2
subplot(2,2,2);
plot(plt.t, plt.p(4:6,:),'LineWidth', 2);
xlabel( 'time [s]');
ylabel('orientation [rad]')
legend('r','p','y');
title('Orientation');
% Vehicle-Base 2,1
subplot(2,2,3);
plot(plt.t, plt.p_dot(1:3,:),'LineWidth', 2);
xlabel( 'time [s]');
ylabel('velocity linear [m/s]')
legend('x', 'y','z');
title('Linear velocity');
% Vehicle-Base 2,2
subplot(2,2,4);
plot(plt.t, plt.p_dot(4:6,:),'LineWidth', 2);
xlabel( 'time [s]');
ylabel('velocity angular [rad/s]')
legend('w_x','w_y','w_z');
title('Angular velocity');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Vehicle Base Error Plot %%%%%%%%%%%%%%%%
figure('Name','Vehicle-Base-Error');
subplot(2,1,1);
plot(plt.t,plt.totalError(1:3,:),'LineWidth', 2);
xlabel( 'time [s]');
ylabel('linear error [m]');
legend('x','y','z');
title('Linear Error of Vehicle Base');

subplot(2,1,2);
plot(plt.t,plt.totalError(4:6,:),'LineWidth', 2);
% set(P2, 'LineWidth', 2);
xlabel( 'time [s]');
ylabel('angular error [rad]');
legend('r','p','y');
title('Angular Error of Vehicle Base');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure('Name','Manipulator-Error');
subplot(2,1,1);

plot(plt.t,plt.toolFrameError(1:3,:),'LineWidth', 2); % 6xn
xlabel('time [s]');
ylabel('linear error [m]');
legend('x','y','z');
title('Tool Frame Position Error');

subplot(2,1,2);
plot(plt.t,plt.toolFrameError(4:6,:),'LineWidth', 2);
xlabel('time [s]');
ylabel('angular error [rad]');
legend('r','p','y');
title('Tool Frame Orientation Error');



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% figure('Name','Tool Frame Error');
% hplot = plot(plt.t,plt.toolFrameError(1:3,:)); % 6xn
% set(hplot, 'LineWidth', 2);
% legend('vechicle disp from floor');
% 
% disp('PrintPlot here 2');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Name','Mac Floor Distance');
% subplot(2,1,1);
% plot(plt.t,plt.totalError(3,:),'LineWidth', 2); % 6xn
% hold on
% plot(plt.t,plt.p(3,:),'LineWidth', 2); % 6xn
% hold off
% xlabel('time [s]');
% ylabel('[m]');
% legend('z error','z pos');
% title('Vehicle Z error and position');

% subplot(2,1,1);
% plot(plt.t, plt.a(10,:),'LineWidth', 2);
% legend('Amac');
% title('Activations');
% xlabel('time [s]');
% ylabel('Activation')
% 

% subplot(2,1,2);
plot(plt.t,plt.distanceFloor,'LineWidth', 2); %
legend('hActual');
title('Vehicle Altitude');
xlabel( 'time [s]');
ylabel('distance [m]')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% exercise 1.2 
figure('Name','MAC');

% subplot(3,1,1);
% plot(plt.t, plt.a(10,:),'LineWidth', 2);
% legend('Amac');
% title('Activations');
% xlabel('time [s]');
% ylabel('Activation')


% subplot(3,1,2);
subplot(2,1,1);
plot(plt.t,plt.distanceFloor,'LineWidth', 2); %
legend('hActual');
title('Vehicle Altitude');
xlabel('time [s]');
ylabel('distance [m]')

% subplot(3,1,3);
subplot(2,1,2);
plot(plt.t, plt.p_dot(3,:),'LineWidth', 2);
xlabel('time [s]');
ylabel('[m/s]');
legend('zdot');
title('Vehicle Z Velocity');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Name','Activations');
sgtitle('Activations for Vehicle');
plot(plt.t, plt.a(8:9,:),'LineWidth', 2);
hold on 
plot(plt.t, plt.a_mp(1:4,:),'LineWidth', 2);


hold off
legend('Amu','Aha','Amp1','Amp2','Amp3','Amp4');
xlabel( 'time [s]');
ylabel('Activation')


end

