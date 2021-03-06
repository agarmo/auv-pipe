clear all;
close all;

global Mass D T eta0 nu0 P0 h Q R focus zg W Xuu North East Down G pipeline WP bottom

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

WP = [];

for i = 1:200:size(pipeline, 1)
    WP = [WP pipeline(i, 1:2)'];
end

bottom = 4;

focus = 1; % camera focus
eta0 = [-100 -100 0 0 0 0]'; %initial position
nu0 = zeros(6,1); %initial velocity

% % kalman filter parameters
% P0 = eye(24);
% h = 0.1; % sampling interval
% Q = diag([1 1 1 1 1 1 .001 .001 .001 .001 .001 .001 10 10 10 10 10 10 .01 .01 .01 .01 .01 .01]);
% % Q = 1.*eye(24);
% % R = 100.*eye(12);
% R = diag([0.1 0.1 0.1 0.1 0.1 0.1 .01 .01 .01 .01 .01 .01]);

% kalman filter 2 parameters
P0 = 0.1.*eye(12);
h = 0.1; % sampling interval
% Q = 10.*eye(12);
% Q = 1.*diag([10 10 10 10 10 10 .01 .01 .01 .01 .01 .01]);
Q = 10.*eye(6);
R = 0.01.*eye(6);
% R = diag([1 .1 1 .1 1 .1]);



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
     
     
%bias parameter

T = 1000*eye(6);

sim los_test_with_camsim
plots
