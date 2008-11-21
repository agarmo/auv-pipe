function y = kalmanfilter3(u)

global n e W Q P0 x0 focus

persistent P_apr x_apr temp

if isempty(P_apr) || isempty(x_apr) || isempty(temp)
    P_apr = P0;
    x_apr = x0;
    temp = [0; 0];
end

p = u(1:6);

eta = u(7:12);
nu = u(14:19);
z_c = u(13);

output = u(20);
t = u(21);
z_b = z_c + eta(3);

psi = eta(6);

R = [cos(psi) -sin(psi) 0;
     sin(psi) cos(psi) 0;
     0          0       1];

%% Kalman filter
%oppdater estimat

%modell av pipeline

u = [(R(1,1:2)*nu(1:2)); R(1:2, 1:2)'*eta(1:2)];

% model equations
t1 = 1/2000;
t2 = 1/2000;
A = [0 0 -t1 0;
     0 0 0 -t2;
     0 0 -t1 0;
     0 0 0 -t2];
B = [n, 0, 0;
     e, 0, 0;
     0, 0, 0;
     0, 0, 0];
f = focus;

C = [(f/z_c)*cos(psi), (f/z_c)*sin(psi), 0, 0;
     -f/z_c*sin(psi), f/z_c*cos(psi), 0, 0];

D = [0, -f/z_c*cos(psi), -f/z_c*sin(psi);
     0, f/z_c*sin(psi), -f/z_c*cos(psi)];
     
E = [0, 0;
     0, 0;
    eye(2)];

[Ad, Bd] = c2d(A, B, 0.1);
[Ad, Ed] = c2d(A, E, 0.1);
x = [];

%     x_apr = [n*eta(1);
%              e*eta(1);
%              x_apr(3);
%              x_apr(4)];

            
    K = P_apr*C'*inv((C*P_apr*C' + W));
for i = 1:2:6
    x_post = x_apr + K*([p(i); p((i+1))] - C*x_apr - D*u);
    x = [x; x_post(1:4)];
end

    P_post = (eye(4) - K*C)*P_apr*(eye(4) - K*C)' + K*W*K';

x_aprm = [];
B = [n;
     e;
     0;
     0];
for i = 1:4:12
    %predict
    x_apr = A*x(i:i+3) + B*eta(1);
    x_aprm = [x_aprm; x_apr];
end
   
P_apr = A*P_post*A' + Ed*Q*Ed';

if temp ~= zeros(2,1)
    pipeline_heading = atan2(abs(x(2)-temp(2)), abs(x(1)-temp(1))); %use prior estimate
  
else
    pipeline_heading = 0;
end

if mod(t,20) == 0
    temp = x(9:10);
end



%% output
y = [x_apr(1:2); z_b; x(1:2); z_b;x(5:6); z_b; x(9:10); z_b; pipeline_heading; diag(P_apr)];

