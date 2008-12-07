clear all;

global pipeline focus WP W Q P0 x0 n e pipeline_dir

N = 0:0.1:1000;
n = 1;
E = 4.*N;
e = 4;
pipeline2 = [N(1:300) N(500:10000); E(1:300) E(500:10000); 10.*ones(1,9801)]';
pipeline = [N(500:10000); E(500:10000); 10.*ones(1,9501)]';


% pipeline = [N; E; 10.*ones(1,10001)];


eta_ref = [100 1/1000*1000 10 0 0 0];

%kalman filter
Q = 10.*eye(2);
W = .01.*eye(2);

P0 = diag([10 10 .1 .1]);

x0 = [0, 0, 0, 0]';


focus = 1;
WP = [];

for i = 1:500:size(pipeline2, 1)
    WP = [WP pipeline2(i, 1:2)'];
end
% WP = [0 10 20 ; 0 10 20];

pipeline_dir = atan2(WP(2, 3)- WP(2, 1), WP(1, 3) - WP(1,1));
% pipeline_dir = 75*pi/180;
sim last_test2
