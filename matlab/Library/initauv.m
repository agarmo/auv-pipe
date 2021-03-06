clear all;
close all;

global M D T eta0 nu0 P0 h Q R focus zg W Xuu North East Down bottom G L pipeline

%Pipeline Coordinates
North = [-10 -5 10];
East = [-10 5 10];
Down = [10 10 10];

%% interpolation of pipeline
N = North;
E = East;
D = Down;

%interpolate the waypoints of the pipeline
t_s = -10:0.1:10;
pipeline_xy = pchip(N, E, t_s);
pipeline_xz = pchip(N, D, t_s);
temp2 = size(t_s');
pipeline = [t_s', pipeline_xy', pipeline_xz'];



bottom = 10;

focus = 1; % camera focus
eta0 = [-10 -5 6 0 0 0]'; %initial position
nu0 = zeros(6,1); %initial velocity

h = 0.5; % sampling interval

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


M = [m-Xu   0       0       0       m*zg    -m*yg;
     0      m-Yv    0       -m*zg   0       m*xg-Yr;
     0      0       m-Zw    m*yg    -m*xg-Zq 0;
     0      -m*zg   m*yg    Ix      0        0;
     m*zg   0       -m*xg-Zq 0      Iy-Mq    0;
     -m*yg  m*xg-Yr 0       0       0       Iz-Nr];


z = 10-7;
     
