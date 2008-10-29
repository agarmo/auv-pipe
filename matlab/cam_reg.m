function K = cam_reg(input) 

global focus bottom
m = 1380;

Iz = 1247;

Xu = -85.57;
Xv = 0;
Yv = -1092.9;
Yr = -84.1448;
Nr = -795.1686;

u = 1;
z = bottom;

Ydv = -931.9715;
Ydr = 1915.8;
Ndv = -1760.7;
Ndr = -8386.3;

gamma = m*(Iz-Nr) + Yv*(Nr - Iz) - Yr^2;

a12 = -gamma*focus/z;
a22 = -(Iz - Nr)*Ydv -Yr*Ndr - Yr*(Xu - Yv)*u;
a23 = -(Iz -Nr)*Ydr - Yr*Ndr -(Iz - Nr)*(m-Xv)*u;
a32 = -Yr*Ydv - (m - Yv)*Ndv - (m-Yv)*(Xu - Yv)*u;
a33 = -Yr*Ydr -(m -Yv)*Ndr - (m - Xu)*Ydr*u;

A = [0 a12 0;
     0 a22 a23;
     0 a32 a33];

B = [0 0;
     (Iz -Nr)/gamma 0;
     0 (m-Yv)/gamma];
 
Q = .001.*eye(3);
R = 1.*eye(2);

[K, S, e] = lqr(A, B, Q, R)

end