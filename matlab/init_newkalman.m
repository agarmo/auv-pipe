clear all;

global pipeline focus WP

N = 0:0.01:100;

E = 1/1000.*N.^3;

pipeline = [N; E; 10.*ones(1,10001)]';


eta_ref = [100 1/1000*1000 10 0 0 0];

focus = 2;
WP = [];

for i = 1:200:size(pipeline, 1)
    WP = [WP pipeline(i, 1:2)'];
end



sim new_kalman