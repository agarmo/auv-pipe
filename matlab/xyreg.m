function tau = xyreg(u)

global Mass D C_xy

M_xy = Mass(1:2, 1:2);
D_xy = D(1:2, 1:2);
C_xy = zeros(2);

eta_d = u(1:2);
eta_d_dot = u(3:4);
eta_d_ddot = u(5:6);
eta = u(7:8);
nu = u(9:10);
b = u(11:12);
psi = u(13);
r = u(14);

K1 = 1*eye(2);
K2 = 1*eye(2);

R = [cos(psi) -sin(psi);
     sin(psi) cos(psi)];
S = [0 -r;
     r 0];

z1 = R'*(eta - eta_d);
z1_dot = -S*R'*(eta - eta_d) + R'*(nu - eta_d_dot);

alpha = R'*eta_d_dot - K1*z1;
alpha_dot = -S*R'*eta_d_dot + R'*eta_d_ddot - K1*z1_dot;

z2 = nu - alpha;

tau = M_xy*alpha_dot - R'*b + C_xy*nu + D_xy*nu - z1 - K2*z2;
 
end
