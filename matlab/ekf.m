function [x] = ekf(u)

global Mass D T eta0 nu0 P0 h Q R focus bottom zg W Xuu

N = 10;
M = N;

persistent x_apr P_apr



if isempty(x_apr)
    x_apr = [eta0;
             [-10, -10, -10, -10, -10, -10]';
             zeros(6,1);
             nu0];
end
if isempty(P_apr)
    P_apr = P0;
end
    



%% Position measurement
e = u(1);
n = u(2);
d = u(3);


tau = u(13:18);



%% Model parameters



% ha to m�le matriser, en n�r det er en pipeline � observere og en n�r det
% ikke er det.
H = [eye(6) zeros(6, 18);
     zeros(6) eye(6) zeros(6, 12)];

     

%% Update estimate
 % problem med kalman gain, matrisen blir ikke inverterbar!
K = P_apr*H'*inv(H*P_apr*H'+R); %compute Kalman gain
x_post = x_apr + K*(u(1:12) - H*x_apr); %State estimate update
P_post = (eye(24) - K*H)*P_apr*(eye(24) - K*H)' + K*R*K'; %Error covariance update

psi = x_post(4);
theta = x_post(5);
phi = x_post(6);

Rot = [cos(psi)*cos(phi) -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi) sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);
     sin(psi)*cos(theta) cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi) -cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi);
     -sin(theta)            -cos(theta)*sin(phi)                            cos(theta)*cos(phi)];

Trans = [1                  sin(phi)*tan(theta)                                 cos(phi)*tan(theta);
     0                  cos(phi)                                            -sin(phi);
     0                  sin(phi)/cos(theta)                                 cos(phi)/cos(theta)];
     
J = [Rot      zeros(3);
     zeros(3)   Trans];

G = [zeros(3,1);
     zg*W*cos(theta)*sin(phi);
     zg*W*sin(theta);
     0];

     
%% interaction matrix
z = bottom - d;

p1x = x_post(7);
p1y = x_post(8);
p2x = x_post(9);
p2y = x_post(10);
p3x = x_post(11);
p3y = x_post(12);


L = [-focus/z, 0, p1x/z p1x*p1y/focus, -(p1x^2)/focus-focus, p1y;
     0, -focus/z, p1y/z, (p1y^2)/focus+focus, -p1x*p1y/focus, -p1x;
     -focus/z, 0, p2x/z, p2x*p2y/focus, -(p2x^2)/focus-focus, p2y;
     0, -focus/z, p2y/z, (p2y^2)/focus+focus, -p2x*p2y/focus, -p2x;
     -focus/z, 0, p3x/z, p3x*p3y/focus, -(p3x^2)/focus-focus, p3y;
     0, -focus/z, p3y/z, (p3y^2)/focus+focus, -p3x*p3y/focus, -p3x];

 
 

%% Predict forward

D_nl = D + (Xuu*abs(x_post(7))).*diag([1 0 0 0 0 0]); %nonlinear damping

x(1:6) = x_post(1:6) + h.*(J*x_post(19:24)); %eta
x(7:12) = x_post(7:12) + h.*(L*x_post(19:24)); %point speed
x(13:18) = x_post(13:18) + h.*(-inv(T)*x_post(13:18)); %bias
x(19:24) = x_post(19:24) + h.*inv(M)*(inv(J)*x_post(13:18) - D_nl*x_post(19:24) - G + tau); %velocity

x_apr = x'; %remember untill next time

%% jacboian of f
u = x_post(7);
v = x_post(8);
w = x_post(9);
p = x_post(10);
q = x_post(11);
r = x_post(12);

jacobiJ = [0,0,0,-cos(psi)*sin(phi)*u+(sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta))*v+(sin(psi)*cos(phi)-cos(psi)*sin(theta)*sin(phi))*w,cos(psi)*cos(theta)*sin(phi)*v+cos(psi)*cos(phi)*cos(theta)*w,  -sin(psi)*cos(phi)*u+(-cos(psi)*cos(phi)-sin(phi)*sin(theta)*sin(psi))*v+(cos(psi)*sin(phi)-sin(theta)*sin(psi)*cos(phi))*w;
           0,0,0,(-cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi))*v+(-cos(psi)*cos(phi)-sin(phi)*sin(theta)*sin(psi))*w,-sin(psi)*sin(theta)*u+sin(phi)*cos(theta)*sin(psi)*v+cos(theta)*sin(psi)*cos(phi)*w, cos(psi)*cos(theta)*u+(-sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi))*v+(sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta))*w;
           0,0,0,-cos(theta)*cos(phi)*v-cos(theta)*sin(phi)*w,-cos(theta)*u+sin(theta)*sin(phi)*v-sin(theta)*cos(phi)*w,0;
           0,0,0,cos(phi)*tan(theta)*q-sin(phi)*tan(theta)*r,sin(phi)*(1+tan(theta)^2)*q+cos(phi)*(1+tan(theta)^2)*r,0;
           0,0,0,-sin(phi)*q-cos(phi)*r,0,0;
           0,0,0,cos(phi)/cos(theta)*q-sin(phi)/cos(theta)*r,sin(phi)/cos(theta)^2*q*sin(theta)+cos(phi)/cos(theta)^2*r*sin(theta),0];

% jacobiL = [1/z*w+p1y/focus*p-2*p1x/focus*q,p1x/focus*p+r,0,0,0,0;
%            -p1y/focus*q-r, 1/z*w+2*p1y/focus*p-p1x/focus*q,0,0,0,0;
%            0,0, 1/z*w+p2y/focus*p-2*p2x/focus*q,p2x/focus*p+r,0,0;
%            0,0,-p2y/focus*q-r, 1/z*w+2*p2y/focus*p-p2x/focus*q,0,0;
%            0,0,0,0, 1/z*w+p3y/focus*p-2*p3x/focus*q,p3x/focus*p+r;
%            0,0,0,0,-p3y/focus*q-r, 1/z*w+2*p3y/focus*p-p3x/focus*q];

% N og M er st�rrelsen p� bilde. alts� vil vi at x3i skal v�re lengst ut
% mot kantent og x1i skal v�re p� motsatt side. Samme med y koordinatene.
% x2i skal v�re i sentrum, ideelt. 

L1_star = [zeros(2, 6);
           0 0 1/z 0 0 0;
           0 0 0   0 0 0;
           0 0 0 0 -2/focus*N 1;
           0 0 0 0 -M/focus 0];
       
L2_star =  [-focus/z, 0, -N/z -N*-M/focus, -(N^2)/focus-focus, -M;
            0, -focus/z, -M/z, (M^2)/focus+focus, N*-M/focus, N;
            -focus/z, 0, 0/z, 0/focus, -(0^2)/focus-focus, 0;
            0, -focus/z, 0/z, (0^2)/focus+focus, -0/focus, -0;
            -focus/z, 0, N/z, N*M/focus, -(N^2)/focus-focus, M;
            0, -focus/z, M/z, (M^2)/focus+focus, -N*M/focus, -N];


PHI = eye(24) + h.*[zeros(6,18) eye(6);
                    zeros(6) L1_star zeros(6) L2_star;
                    zeros(6,12) -inv(T) zeros(6);
                    zeros(6,12) eye(6) -inv(M)*D_nl];

              
GAMMA = h.*[zeros(6,24);
            zeros(6), 10.*eye(6), zeros(6, 12);
            zeros(6,12), eye(6), zeros(6);
            zeros(6, 18), inv(Mass)];

%% Error Covariance propagation

P_apr = PHI*P_post*PHI' + GAMMA*Q*GAMMA';


end
