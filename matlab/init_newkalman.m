clear all;

global pipeline focus WP W Q P0 x0 n e

N = 0:0.1:1000;
n = 1;
E = 4.*N;
e = 4;
pipeline = [N; E; 10.*ones(1,10001)]';


eta_ref = [100 1/1000*1000 10 0 0 0];

%kalman filter
Q = 10.*eye(2);
W = 0.01.*eye(2);

P0 = diag([10 10 0.1 .1]);

x0 = [0, 0, 0, 0]';


focus = 1;
WP = [];

for i = 1:500:size(pipeline, 1)
    WP = [WP pipeline(i, 1:2)'];
end



sim last_test
