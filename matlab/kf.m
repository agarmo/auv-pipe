function x = kf(u)



global M G D T L Q R eta0 nu0 P0 h

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

%transform to VP

psi = u(6);

Rot = [cos(psi) -sin(psi) 0;
     sin(psi) cos(psi)  0;
     0          0       1];

Prot = [Rot zeros(3);
     zeros(3) eye(3)];
     
eta_vp = Prot'*u(1:6);

H = [eye(6) zeros(6, 18);
     zeros(6) eye(6) zeros(6, 12)];


     
K = P_apr*H'*inv(H*P_apr*H'+R); %compute Kalman gain
x_post = x_apr + K*([eta_vp; u(7:12)] - H*x_apr);

P_post = (eye(24) - K*H)*P_apr*(eye(24) - K*H)' + K*R*K';


%% Predict forward

tau = u(13:18);

%system matrix
A = [zeros(6, 18) eye(6);
     zeros(6,18) L;
     zeros(6,12) -inv(T) zeros(6);
     -inv(M)*G zeros(6) eye(6) -inv(M)*D];

B = [zeros(18,6);
     inv(M)];
     
E = [zeros(6,24);
     zeros(6) eye(6) zeros(6, 12);
     zeros(6, 12) eye(6) zeros(6);
     zeros(6, 18) inv(M)];
     
[PHI, DELTA] = c2d(A, B, h);
[PHI, GAMMA] = c2d(A, E, h);

     
x_apr = PHI*x_post + DELTA*tau;

x(1:6) = Prot*x_apr(1:6);
x(7:12) = x_apr(7:12);
x(13:18) = Prot*x_apr(13:18);
x(19:24) = x_apr(19:24);

P_apr = PHI*P_post*PHI' + GAMMA*Q*GAMMA';


