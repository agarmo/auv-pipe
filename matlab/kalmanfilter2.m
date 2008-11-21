function y = kalmanfilter(u)

global n e W Q P0 x0 focus

persistent P_apr x_apr

if isempty(P_apr) || isempty(x_apr)
    P_apr = P0;
    x_apr = x0;
end

p = u(1:6);

eta = u(7:12);
nu = u(14:19);
z_c = u(13);

output = u(20);

z_b = z_c + eta(3);

psi = eta(6);

R = [cos(psi) -sin(psi) 0;
     sin(psi) cos(psi) 0;
     0          0       1];

%% Kalman filter
%oppdater estimat

%modell av pipeline

u = [(R(1,1:2)*nu(1:2)); R(1:2, 1:2)'*eta(1:2)];

t1 = 1/1000;
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

    K = P_apr*C'*inv((C*P_apr*C' + W));
    x_post = x_apr + K*([p(1); p(2)] - C*x_apr - D*u);

    P_post = (eye(4) - K*C)*P_apr*(eye(4) - K*C)' + K*W*K';

    %predict

    x_apr = Ad*x_apr + Bd*u;

    P_apr = Ad*P_post*Ad' + Ed*Q*Ed';

%% output
y = [x_apr(1:2); z_b; x_post(1:2); z_b;];

