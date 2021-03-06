function [x] = ekf2(u)

global Mass D T eta0 P0 h Q R focus zg W Xuu


persistent x_apr P_apr



if isempty(x_apr)
    x_apr = [eta0; zeros(6,1)];
end
if isempty(P_apr)
    P_apr = P0;
end

% camOutput = 1;


%% Position measurement


eta = u(1:6);
xi = u(7:12);
tau = u(13:18);
nu = u(19:24);
nu_dot = u(25:30);
bottom = u(31);
camOutput = u(32);




if camOutput == 1  

%% Model parameters



% ha to målematriser, en n�r det er en pipeline � observere og en n�r det
% ikke er det.
    H = [eye(6) zeros(6)];

%% Update estimate
 % problem med kalman gain, matrisen blir ikke inverterbar!
K = P_apr*H'*inv(H*P_apr*H' + R); %compute Kalman gain
x_post = x_apr + K*(xi - H*x_apr); %State estimate update %Må ha annen update når Pipeline ikke er tilgjengelig
P_post = (eye(12) - K*H)*P_apr*(eye(12) - K*H)' + K*R*K'; %Error covariance update

phi = eta(4);
theta = eta(5);
psi = eta(6);

Rot = [cos(psi)*cos(theta) -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi) sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);
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
z = bottom - eta(3);

p1x = x_post(1);
p1y = x_post(2);
p2x = x_post(3);
p2y = x_post(4);
p3x = x_post(5);
p3y = x_post(6);


L = [-focus/z, 0, p1x/z p1x*p1y/focus, -(p1x^2)/focus-focus, p1y;
     0, -focus/z, p1y/z, (p1y^2)/focus+focus, -p1x*p1y/focus, -p1x;
     -focus/z, 0, p2x/z, p2x*p2y/focus, -(p2x^2)/focus-focus, p2y;
     0, -focus/z, p2y/z, (p2y^2)/focus+focus, -p2x*p2y/focus, -p2x;
     -focus/z, 0, p3x/z, p3x*p3y/focus, -(p3x^2)/focus-focus, p3y;
     0, -focus/z, p3y/z, (p3y^2)/focus+focus, -p3x*p3y/focus, -p3x];

 
 

%% Predict forward

D_nl = D + (Xuu*abs(nu(1))).*diag([1 0 0 0 0 0]); %nonlinear damping

x(1:6) = x_post(1:6) + h.*(L*nu); %point speed
x(7:12) = x_post(7:12) + h.*(inv(T)*J*(-G + tau - D_nl*nu - Mass*nu_dot)); %bias

x_apr = x'; %remember untill next time

%% jacboian of f
u = nu(1);
v = nu(2);
w = nu(3);
p = nu(4);
q = nu(5);
r = nu(6);

% jacobiJ_eta = [0,0,0,(sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta))*v+(sin(psi)*cos(phi)-cos(psi)*sin(theta)*sin(phi))*w,-cos(psi)*sin(theta)*u+cos(psi)*cos(theta)*sin(phi)*v+cos(psi)*cos(phi)*cos(theta)*w, -sin(psi)*cos(theta)*u+(-cos(psi)*cos(phi)-sin(phi)*sin(theta)*sin(psi))*v+(cos(psi)*sin(phi)-sin(theta)*sin(psi)*cos(phi))*w;
%                0,0,0,(-cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi))*v+(-cos(psi)*cos(phi)-sin(phi)*sin(theta)*sin(psi))*w,-sin(psi)*sin(theta)*u+sin(phi)*cos(theta)*sin(psi)*v+cos(theta)*sin(psi)*cos(phi)*w,  cos(psi)*cos(theta)*u+(-sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi))*v+(sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta))*w;
%                0,0,0,-cos(theta)*cos(phi)*v-cos(theta)*sin(phi)*w,-cos(theta)*u+sin(theta)*sin(phi)*v-sin(theta)*cos(phi)*w,0;
%                0,0,0,cos(phi)*tan(theta)*q-sin(phi)*tan(theta)*r,sin(phi)*(1+tan(theta)^2)*q+cos(phi)*(1+tan(theta)^2)*r,0;
%                0,0,0,-sin(phi)*q-cos(phi)*r,0,0;
%                0,0,0,cos(phi)/cos(theta)*q-sin(phi)/cos(theta)*r,sin(phi)/cos(theta)^2*q*sin(theta)+cos(phi)/cos(theta)^2*r*sin(theta),0];
%  
% 
% jacobiJ_nu =  [cos(psi)*cos(theta), -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi),  sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta),0,0,0;
%                sin(psi)*cos(theta),  cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi), -cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi),0,0,0;
%                -sin(theta),-cos(theta)*sin(phi),cos(theta)*cos(phi),0,0,0;
%                0,0,0,1,sin(phi)*tan(theta),cos(phi)*tan(theta);
%                0,0,0,0,cos(phi),-sin(phi);
%                0,0,0,0,sin(phi)/cos(theta),cos(phi)/cos(theta)];
%  
%            
% inv_jacobiJ_b =   [cos(theta)*(cos(psi)*cos(phi)^2+2*sin(phi)*sin(theta)*sin(psi)*cos(phi)-cos(psi)*sin(phi)^2)/(cos(psi)^2*cos(theta)^2*cos(phi)^2-cos(psi)^2*cos(theta)^2*sin(phi)^2+sin(psi)^2*cos(theta)^2*cos(phi)^2-sin(psi)^2*cos(theta)^2*sin(phi)^2+sin(theta)^2*sin(psi)^2*cos(phi)^2+sin(theta)^2*cos(psi)^2*sin(phi)^2+sin(theta)^2*cos(psi)^2*cos(phi)^2+sin(theta)^2*sin(phi)^2*sin(psi)^2)*(-1/b1*conj(b1)+2*abs(1,b1)/signum(b1)), -cos(theta)*(-sin(psi)*cos(phi)^2+2*cos(psi)*sin(theta)*sin(phi)*cos(phi)+sin(psi)*sin(phi)^2)/(cos(psi)^2*cos(theta)^2*cos(phi)^2-cos(psi)^2*cos(theta)^2*sin(phi)^2+sin(psi)^2*cos(theta)^2*cos(phi)^2-sin(psi)^2*cos(theta)^2*sin(phi)^2+sin(theta)^2*sin(psi)^2*cos(phi)^2+sin(theta)^2*cos(psi)^2*sin(phi)^2+sin(theta)^2*cos(psi)^2*cos(phi)^2+sin(theta)^2*sin(phi)^2*sin(psi)^2)*(-1/b2*conj(b2)+2*abs(1,b2)/signum(b2)),                                                                                                                                                                                                                                                    -(cos(phi)^2+sin(phi)^2)*sin(theta)/(cos(theta)^2*cos(phi)^2-cos(theta)^2*sin(phi)^2+sin(theta)^2*sin(phi)^2+sin(theta)^2*cos(phi)^2)*(-1/b3*conj(b3)+2*abs(1,b3)/signum(b3)),0,0,0;
%               (-sin(psi)*cos(theta)^2*cos(phi)+cos(psi)*sin(theta)*sin(phi)-sin(theta)^2*sin(psi)*cos(phi))/(cos(psi)^2*cos(theta)^2*cos(phi)^2-cos(psi)^2*cos(theta)^2*sin(phi)^2+sin(psi)^2*cos(theta)^2*cos(phi)^2-sin(psi)^2*cos(theta)^2*sin(phi)^2+sin(theta)^2*sin(psi)^2*cos(phi)^2+sin(theta)^2*cos(psi)^2*sin(phi)^2+sin(theta)^2*cos(psi)^2*cos(phi)^2+sin(theta)^2*sin(phi)^2*sin(psi)^2)*(-1/b1*conj(b1)+2*abs(1,b1)/signum(b1)),   (cos(psi)*cos(theta)^2*cos(phi)+sin(phi)*sin(theta)*sin(psi)+cos(psi)*cos(phi)*sin(theta)^2)/(cos(psi)^2*cos(theta)^2*cos(phi)^2-cos(psi)^2*cos(theta)^2*sin(phi)^2+sin(psi)^2*cos(theta)^2*cos(phi)^2-sin(psi)^2*cos(theta)^2*sin(phi)^2+sin(theta)^2*sin(psi)^2*cos(phi)^2+sin(theta)^2*cos(psi)^2*sin(phi)^2+sin(theta)^2*cos(psi)^2*cos(phi)^2+sin(theta)^2*sin(phi)^2*sin(psi)^2)*(-1/b2*conj(b2)+2*abs(1,b2)/signum(b2)),                                                                                                                                                                                                                                                                    cos(theta)*sin(phi)/(cos(theta)^2*cos(phi)^2-cos(theta)^2*sin(phi)^2+sin(theta)^2*sin(phi)^2+sin(theta)^2*cos(phi)^2)*(-1/b3*conj(b3)+2*abs(1,b3)/signum(b3)),0,0,0;
%               (-sin(psi)*cos(theta)^2*sin(phi)+cos(psi)*cos(phi)*sin(theta)+sin(phi)*sin(theta)^2*sin(psi))/(cos(psi)^2*cos(theta)^2*cos(phi)^2-cos(psi)^2*cos(theta)^2*sin(phi)^2+sin(psi)^2*cos(theta)^2*cos(phi)^2-sin(psi)^2*cos(theta)^2*sin(phi)^2+sin(theta)^2*sin(psi)^2*cos(phi)^2+sin(theta)^2*cos(psi)^2*sin(phi)^2+sin(theta)^2*cos(psi)^2*cos(phi)^2+sin(theta)^2*sin(phi)^2*sin(psi)^2)*(-1/b1*conj(b1)+2*abs(1,b1)/signum(b1)),   (cos(psi)*cos(theta)^2*sin(phi)+sin(theta)*sin(psi)*cos(phi)-cos(psi)*sin(theta)^2*sin(phi))/(cos(psi)^2*cos(theta)^2*cos(phi)^2-cos(psi)^2*cos(theta)^2*sin(phi)^2+sin(psi)^2*cos(theta)^2*cos(phi)^2-sin(psi)^2*cos(theta)^2*sin(phi)^2+sin(theta)^2*sin(psi)^2*cos(phi)^2+sin(theta)^2*cos(psi)^2*sin(phi)^2+sin(theta)^2*cos(psi)^2*cos(phi)^2+sin(theta)^2*sin(phi)^2*sin(psi)^2)*(-1/b2*conj(b2)+2*abs(1,b2)/signum(b2)),                                                                                                                                                                                                                                                                    cos(theta)*cos(phi)/(cos(theta)^2*cos(phi)^2-cos(theta)^2*sin(phi)^2+sin(theta)^2*sin(phi)^2+sin(theta)^2*cos(phi)^2)*(-1/b3*conj(b3)+2*abs(1,b3)/signum(b3)),0,0,0;
%                0,0,0,-1/b4*conj(b4)+2*abs(1,b4)/signum(b4),0,-cos(theta)*tan(theta)*(-1/b6*conj(b6)+2*abs(1,b6)/signum(b6));
%                0,0,0,0,cos(phi)/(cos(phi)^2+sin(phi)^2)*(-1/b5*conj(b5)+2*abs(1,b5)/signum(b5)),                                                                                                                                                                                                                                                                                                                                              cos(theta)*sin(phi)/(cos(phi)^2+sin(phi)^2)*(-1/b6*conj(b6)+2*abs(1,b6)/signum(b6));
%                0,0,0,0,-sin(phi)/(cos(phi)^2+sin(phi)^2)*(-1/b5*conj(b5)+2*abs(1,b5)/signum(b5)),                                                                                                                                                                                                                                                                                                                                              cos(theta)*cos(phi)/(cos(phi)^2+sin(phi)^2)*(-1/b6*conj(b6)+2*abs(1,b6)/signum(b6))];
%  
% jacobi_b = [0,0,0,0,0,0;
%             0,0,0,0,0,0;
%             0,0,0,0,0,0;
%             0,0,0,1/1000*zg*W*cos(theta)*cos(phi)+1/1000*cos(phi)*tan(theta)*zg*W*sin(theta), -1/1000*zg*W*sin(theta)*sin(phi)+1/1000*sin(phi)*(1+tan(theta)^2)*zg*W*sin(theta)+1/1000*sin(phi)*tan(theta)*zg*W*cos(theta),0;
%             0,0,0,-1/1000*zg*W*sin(theta)*sin(phi),1/1000*zg*W*cos(theta)*cos(phi),0;
%             0,0,0,1/1000*cos(phi)/cos(theta)*zg*W*sin(theta),1/1000*sin(phi)/cos(theta)^2*zg*W*sin(theta)^2+1/1000*sin(phi)*zg*W,0];
%  
           
           
%            
L1_star = [1/z*w+p1y/focus*p-2*p1x/focus*q,p1x/focus*p+r,0,0,0,0;
           -p1y/focus*q-r, 1/z*w+2*p1y/focus*p-p1x/focus*q,0,0,0,0;
           0,0, 1/z*w+p2y/focus*p-2*p2x/focus*q,p2x/focus*p+r,0,0;
           0,0,-p2y/focus*q-r, 1/z*w+2*p2y/focus*p-p2x/focus*q,0,0;
           0,0,0,0, 1/z*w+p3y/focus*p-2*p3x/focus*q,p3x/focus*p+r;
           0,0,0,0,-p3y/focus*q-r, 1/z*w+2*p3y/focus*p-p3x/focus*q];

% 
% L1_star = [zeros(2, 6);
%            0 0 1/z*nu(3) 0 0 0;
%            0 0 0   0 0 0;
%            0 0 0 0 -2/focus*p3x*nu(5) 1*nu(6);
%            0 0 0 0 -p3y/focus*nu(5) 0];
%        
% L2_star =  [-focus/z, 0, p1x/z p1y*p1x/focus, -(p1x^2)/focus-focus, p1y;
%             0, -focus/z, -p1y/z, (p1y^2)/focus+focus, -p1x*p1y/focus, -p1x;
%             -focus/z, 0, p2x/z, p2x*p2y/focus, -(p2x^2)/focus-focus, p2y;
%             0, -focus/z, p2y/z, (p2y^2)/focus+focus, -p2x*p2y/focus, -p2x;
%             -focus/z, 0, p3x/z, p3x*p3y/focus, -(p3x^2)/focus-focus, p3y;
%             0, -focus/z, p3y/z, (p3y^2)/focus+focus, -p3x*p3y/focus, -p3x];


PHI = eye(12)+ h.*[L1_star, zeros(6) ;
                    zeros(6,12)];

GAMMA = h.*[zeros(6);
             eye(6)];
% GAMMA = h.*E;

%% Error Covariance propagation

P_apr = PHI*P_post*PHI' + GAMMA*Q*GAMMA';

else
    x = x_apr;

    
end
end
