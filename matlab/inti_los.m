
clear all;
close all;

global WP


WP = [50 100 500 700;
      50 50  300 -100];
  
sim los_test


plot(eta.signals.values(:,2), eta.signals.values(:,1))
hold on
plot(WP(2,:), WP(1,:), 'r.');

hold off

