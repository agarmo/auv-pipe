clear all;

WP = [[-10 -10 10]' [-10 5 10]' [10 10 10]'];
N = [-10 -5 10];
E = [-10 5 10];
D = [10 10 1];

t = -10:0.1:10;
%plot(N, E);
pipeline_xy = pchip(N, E, t);
pipeline_xz = pchip(N, D, t);

storrelse = size(t);
pipeline = [t', pipeline_xy', pipeline_xz', ones(storrelse(2), 1)];


%convert pipeline into camera coordinates

T = viewmtx(10, -180, 25);

pipeline_c = [];
for i = 1:size(pipeline)
    if isempty(pipeline_c)
        pipeline_c = T*pipeline(i,:)';
    else
        pipeline_c = [pipeline_c T*pipeline(i,:)'];
    end
end
pipeline_c = pipeline_c';
pos_auv = [-10, -9, 6, 1]';

pos_auv_cam = T*pos_auv;

%calculate limits of the field of view

bottom = 10;
depth = bottom-pos_auv(3);
fov = depth*tand(25/2);

if pos_auv_cam(1) < 0
    x_m = pos_auv_cam(1) - fov;
    x_i = pos_auv_cam(1) + fov;
else
    x_m = pos_auv_cam(1) + fov;
    x_i = pos_auv_cam(1) - fov;
end

if pos_auv_cam(2) < 0
    y_m = pos_auv_cam(2) - fov;
    y_i = pos_auv_cam(2) + fov;
else
    y_m = pos_auv_cam(2) + fov;
    y_i = pos_auv_cam(2) - fov;
end
pipeline_inside = [];

for i = 1:size(pipeline_c)
    if (pipeline_c(i,1) < x_m) && (pipeline_c(i,1) > -x_i) %inside x direction
        
        if (pipeline_c(i,2) < y_m) && (pipeline_c(i, 2) > -y_i) %inside y direction
            
            pipeline_inside = [pipeline_inside; pipeline_c(i,:)];
        end
    end
    
end

%transform back

pipeline_inside_NED = [];
for i = 1:size(pipeline_inside)
    if isempty(pipeline_inside_NED)
        pipeline_inside_NED = inv(T)*pipeline_c(i,:)';
    else
        pipeline_inside_NED = [pipeline_inside_NED inv(T)*pipeline_c(i,:)'];
    end
end
pipeline_inside_NED = pipeline_inside_NED';



% figure(1)
plot3(pipeline_inside_NED(:,2), pipeline_inside_NED(:,1), pipeline_inside_NED(:,3))
% hold on

% plot3(pipeline(:,2), pipeline(:,1), pipeline(:,3))

