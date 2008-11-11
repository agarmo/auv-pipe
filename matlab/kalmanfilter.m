function y = kalmanfilter(u)



x1 = u(1);
y1 = u(2);
x2 = u(3);
y2 = u(4);
x3 = u(5);
y3 = u(6);

eta = u(7:12);
nu = u(14:19);
z_c = u(13);

z_b = z_c + eta(3);

psi = eta(6);

R = [cos(psi) -sin(psi) 0;
     sin(psi) cos(psi) 0;
     0          0       1];

%% posisjon i forhold til camera representert i NED

global focus

x1c = x1*z_c/focus;
y1c = y1*z_c/focus;
z1c = z_b;

Pc = [x1c y1c z1c]';

Pwc = R*Pc + eta(1:3); % Måling;






%% Prediktert pipeline


x1w = eta(1);
y1w = 1/1000*eta(1)^3;
z1w = z_b;

Pw = [x1w y1w z1w]';



y1 = Pw - Pwc;
y2 = Pw;
y3 = Pwc;

y = [y1; y2; y3;];

