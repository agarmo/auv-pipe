clear all;

global pipeline focus WP W Q P0 x0 n e

N = 0:0.1:1000;
n = 1;
E = 4.*N;
e = 4;
pipeline = [N(1:300) N(500:10000); E(1:300) E(500:10000); 10.*ones(1,9801)]';


eta_ref = [100 1/1000*1000 10 0 0 0];

%kalman filter
Q = 1.*eye(2);
W = .1.*eye(2);

P0 = diag([1 1 10 10]);

x0 = [0, 0, 0, 0]';


focus = 1;
WP = [];

for i = 1:500:size(pipeline, 1)
    WP = [WP pipeline(i, 1:2)'];
end



sim last_test
