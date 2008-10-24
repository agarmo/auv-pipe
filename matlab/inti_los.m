
% clear all;
close all;

global WP


WP = [50 100 500 700;
      50 50  300 -100];
  
sim los_test


plot(eta.signals.values(:,2), eta.signals.values(:,1))
hold on
plot(WP(2,:), WP(1,:), 'r.');
plot(pipeline(:,2), pipeline(:,1), 'g--');

% plot(eta_beta.signals.values(:,2), eta_beta.signals.values(:,1), 'r')

legend('Position of AUV with current compensation', 'Waypoints', 'Position without Current comensation');
title('NE position of AUV movement with LOS guidance. Angle of current affecting the vessel 45^\circ');
xlabel('East [m]');
ylabel('North [m]')
hold off
% 
