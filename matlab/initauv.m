clear all;
close all;

global Mass D T eta0 nu0 P0 h Q R focus zg W Xuu North East Down G L pipeline WP

North = [10 15 100 300 400];
East = [10 11 14 15 20];
Down = [100 100 100 100 100];
    

%% interpolation of pipeline
N = North;
E = East;
D = Down;

%interpolate the waypoints of the pipeline
t_s = 10:0.1:400;
pipeline_xy = pchip(N, E, t_s);
% pipeline_xz = pchip(N, D, t_s);
pipeline_xz = 60.*ones(1,3901);
temp2 = size(t_s');
pipeline = [t_s', pipeline_xy', pipeline_xz'];

WP = [pipeline(1,1:2)' pipeline(100,1:2)' pipeline(200,1:2)' pipeline(300,1:2)' pipeline(500,1:2)' pipeline(600,1:2)' pipeline(700,1:2)' pipeline(1000,1:2)' [300 15]' [400 20]'];

bottom = 4;

focus = 1; % camera focus
eta0 = [0 0 0 0 0 0]'; %initial position
nu0 = zeros(6,1); %initial velocity

% % kalman filter parameters
% P0 = eye(24);
% h = 0.1; % sampling interval
% Q = diag([1 1 1 1 1 1 .001 .001 .001 .001 .001 .001 10 10 10 10 10 10 .01 .01 .01 .01 .01 .01]);
% % Q = 1.*eye(24);
% % R = 100.*eye(12);
% R = diag([0.1 0.1 0.1 0.1 0.1 0.1 .01 .01 .01 .01 .01 .01]);

% kalman filter 2 parameters
P0 = eye(18);
h = 0.1; % sampling interval
Q = diag([.1 .1 .01 .01 .01 .01 .1 .1 .1 .1 .1 .1]);
% R = .01.*eye(12);
R = diag([10 5 3 2 2 6 .1 .1 .1 .1 .1 .1]);



%model parameters
m = 1380;
xg = 0;
yg = 0;
zg = 0.0328;
Ix = 86.53;
Iy = 1247;
Iz = 1247;

W = m*9.81;

Xu = -85.57;
Yv = -1092.9;
Yr = -84.1448;
Zw = -1092.9;
Zq = 84.1448;
Mq = -795.1686;
Nr = -795.1686;

Xdu = -19.1881;
Ydv = -931.9715;
Ydr = 1915.8;
Zdw = -931.9715;
Zdq = -1915.8;
Kdp = -1000;
Mdw = 1760.7;
Mdq = -8386.3;
Ndv = -1760.7;
Ndr = -8386.3;

Xuu = -3.8938;

D = [Xdu 0 0 0 0 0;
      0 Ydv 0 0 0 Ydr;
      0 0 Zdw 0 Zdq 0;
      0 0 0 Kdp 0 0;
      0 0 Mdw 0 Mdq 0;
      0 Ndv 0 0 0 Ndr];


Mass = [m-Xu   0       0       0       m*zg    -m*yg;
     0      m-Yv    0       -m*zg   0       m*xg-Yr;
     0      0       m-Zw    m*yg    -m*xg-Zq 0;
     0      -m*zg   m*yg    Ix      0        0;
     m*zg   0       -m*xg-Zq 0      Iy-Mq    0;
     -m*yg  m*xg-Yr 0       0       0       Iz-Nr];

G = [zeros(3, 6);
     zeros(3) diag([-zg*W -zg*W 0])];

z = 10-7;
     
L = [zeros(2, 6);
     0 0 1/z -10 0 0;
     0 0 0 2/focus*(-10) 0 0;
     0 0 0 0 -2/focus*(-10) 1;
     0 0 0 0 10/focus 0];
     
     
     
%bias parameter

T = 1000*eye(6);

sim los_test_with_camsim

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