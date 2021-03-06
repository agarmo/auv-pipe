function y = kalmanfilter(u)

global n e W Q P0 x0 focus

persistent P_apr x_apr

if isempty(P_apr) || isempty(x_apr)
    P_apr = P0(1:2,1:2);
    x_apr = x0(1:2);
end
x1 = u(1);
y1 = u(2);
x2 = u(3);
y2 = u(4);
x3 = u(5);
y3 = u(6);

eta = u(7:12);
nu = u(14:19);
z_c = u(13);

output = u(20);

z_b = z_c + eta(3);

psi = eta(6);

%% Prediktert pipeline


x1w = eta(1);
y1w = 4*eta(1);
z1w = z_b;

x1wt = eta(1)-1;
y1wt = 4*(eta(1)-1);

x1wp = eta(1)+3;
y1wp = 4*(eta(1)+3);

Pw= [x1w y1w z1w]';

Pwp = [x1wp; y1wp; z1w];
Pwa = [x1wt; y1wt; z1w];
heading_p = atan2(y1w-y1wt, x1w-x1wt);

Pw1 = [Pw; Pw; Pw];


%% posisjon i forhold til camera representert i NED

if output == 1
    x1c = x1*z_c/focus;
    y1c = y1*z_c/focus;
    z1c = z_c;


    x2c = x2*z_c/focus;
    y2c = y2*z_c/focus;
    z2c = z_c;

    x3c = x3*z_c/focus;
    y3c = y3*z_c/focus;
    z3c = z_c;


    Pc1 = [x1c y1c z1c]';
    Pc2 = [x2c y2c z2c]';
    Pc3 = [x3c y3c z3c]';

    
    Rpw = [cos(heading_p) -sin(heading_p) 0;
         sin(heading_p) cos(heading_p) 0;
            0          0       1];

    R = [cos(psi) -sin(psi) 0;
         sin(psi) cos(psi) 0;
           0          0       1];
        
    
    Pc1w = (R'*Pc1 + eta(1:3));
    Pc2w = (R'*Pc2 + eta(1:3));
    Pc3w = (R'*Pc3 + eta(1:3));

    if Pc1w(1:2) < Pc3w(1:2)
    
        heading_m = atan2(Pc3w(2)-Pc1w(2), Pc3w(1)-Pc1w(1)) ;
    else
        heading_m = atan2(Pc1w(2)-Pc3w(2), Pc1w(1)-Pc3w(1)) ;
    end
    Pcp =[Pc1w; Pc2w; Pc3w];
else
    Pcp = zeros(9,1);
    heading_m = 0;
end

y1 = Pw1 - Pcp;
y2 = Pw;
y3 = Pcp;


%% vektet output
if output == 1
    alpha = 0.9;
else 
    alpha = 1;
end
beta = 1-alpha;
vekt = [Pw;Pw] + beta.*([y1(1:3);y1(7:9)]);

%% output
y = [y1; Pw; y3; heading_p; heading_m; vekt];

