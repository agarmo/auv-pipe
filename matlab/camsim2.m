function [x] = camsim2(u)

%% Initialization
global pipeline

eta = u(1:6);
nu = u(7:12);

bottom = u(13);


%% Rotation matrix
phi = eta(4);
theta = eta(5);
psi = eta(6);

Rot = [cos(psi)*cos(phi) -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi) sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);
     sin(psi)*cos(theta) cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi) -cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi);
     -sin(theta)            -cos(theta)*sin(phi)                            cos(theta)*cos(phi)];

Trans = [1                  sin(phi)*tan(theta)                                 cos(phi)*tan(theta);
     0                  cos(phi)                                            -sin(phi);
     0                  sin(phi)/cos(theta)                                 cos(phi)/cos(theta)];
     
J = [Rot      zeros(3);
     zeros(3)   Trans];

%% Check if pipeline is inside FOV

% determine fov
eta_b = inv(J)*eta; %transform to body coorodinates

z = bottom - eta(3); %altitude
fov = z*tand(25/2);

if eta(1) < 0
    x_max = eta_b(1) - fov;
    x_min = eta_b(1) + fov;
else
    x_max = eta_b(1) + fov;
    x_min = eta_b(1) - fov;
end

if eta(2) < 0
    y_max = eta_b(2) - fov;
    y_min = eta_b(2) + fov;
else
    y_max = eta_b(2) + fov;
    y_min = eta_b(2) - fov;
end

% x_max
% x_min
% y_max
% y_min


temp_max = J*[x_max; y_max; eta_b(3:6)];
temp_min = J*[x_min; y_min; eta_b(3:6)]; 

x_max = temp_max(1);
x_min = temp_min(1);
y_max = temp_max(2);
y_min = temp_min(2);

pipeline_inside = [];
for i = 1:size(pipeline,1)
    if (pipeline(i,1) <= x_max) && (pipeline(i,1) >= x_min) %inside x direction
        if (pipeline(i,2) <= y_max) && (pipeline(i, 2) >= y_min) %inside y direction

            x_max
            x_min
            y_max
            y_min
     
            pipeline_inside = [pipeline_inside; pipeline(i,:)]; %#ok<AGROW>
     
            %         disp('ingen punkter')
        end
    end
end


if isempty(pipeline_inside)
%     disp('ingen punkter i området');
    P = zeros(6,1);
else

     temp = size(pipeline_inside, 1);
     temp3 = ceil(temp/2);
 
    P = [pipeline_inside(1,1:2)';
         pipeline_inside(temp3, 1:2)';
         pipeline_inside(temp,1:2)'];
end

x = P;
end