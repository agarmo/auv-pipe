
figure(1)
plot(eta.signals.values(:,2), eta.signals.values(:,1))
hold on
plot(WP(2,:), WP(1,:), 'r*');
plot(pipeline(:,2), pipeline(:,1), 'r--');

% plot(eta_beta.signals.values(:,2), eta_beta.signals.values(:,1), 'r')

legend('NE trajectory of AUV', 'LOS Guidance Waypoints', 'Pipeline Trajectory');
title('NE position of AUV movement along waypoints with LOS guidance.');
xlabel('East [m]');
ylabel('North [m]')
hold off
% 

figure(2)
subplot(2, 1, 1)
plot(Translation_ref.time, Translation_ref.signals(1,3).values)
xlabel('Time [s]');
ylabel('Down [m]');
title('Depth of AUV versus Reference');
legend('Depth Reference', 'AUV depth');
grid on

subplot(2,1, 2)
plot(Attitude_ref.time, Attitude_ref.signals(1,3).values)
xlabel('Time [s]');
ylabel('Heading [degrees]');
title('Heading reference versus Heading');
legend('\psi_d', '\psi');
grid on


figure(3)
plot3(eta.signals.values(:,2), eta.signals.values(:,1), -eta.signals.values(:,3));
xlabel('East [m]');
ylabel('North [m]');
zlabel('Down [m]');
grid on;
hold on
plot3(pipeline(:,2), pipeline(:,1), -pipeline(:,3), 'r--')
% legend('AUV Trajectory', 'Pipeline Trajectory');
hold off;

figure(4)
plot3(eta.signals.values(:,2), eta.signals.values(:,1), -eta.signals.values(:,3));
xlabel('East [m]');
ylabel('North [m]');
zlabel('Down [m]');
grid on;
hold on
plot3(Camsim_output(:,5), Camsim_output(:,4), -Translation_ref.signals(1,3).values(:,1)-bottom, 'r.')
% legend('AUV Trajectory', 'Observed pipeline trajectory from camera');
hold off;