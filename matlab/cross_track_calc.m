function x = cross_track_calc(u)

global WP %global waypoint vector
persistent current last %current WP

m = 1380;
Iz = 1247;

if isempty(current)
    current = 2;
end
if isempty(last)
    last = 0;
end

%r0 = 6; %circle of acceptance Too big!
r0 = 2;

eta = u(1:6);
nu = u(7:12);

U = nu(1);
r = nu(6);

lambda1 = 0.5;
lambda2 = 0.1;
n = 1;
phi = 0.1;


if current == 1
    psi_d = atan2(WP(2, current)-eta(2), WP(1, current)-eta(1));
else
    psi_los = atan2(WP(2, current)-eta(2), WP(1, current)-eta(1));
    delta_p = eta(6) - psi_los;
    Li = sqrt((WP(1, current)-WP(1,current-1))^2 + (WP(2, current)-WP(2, current-1))^2);
    S = (WP(1, current)-eta(1)*(WP(1, current)-WP(1, current-1)) + (WP(2, current)-eta(2)*(WP(2, current)-WP(2, current-1))))/Li;
    psi_track = atan2(WP(2, current)-WP(2, current-1), WP(1, current)-WP(1,current-1));
    psi_cte = eta(6) - psi_track;
    sigma = U*r*cos(psi_cte) + lambda1*U*sin(psi_cte) + lambda2*S*sin(delta_p);
    if cos(psi_cte) == 0
        tau6 = 0;
    else
       tau6 = Iz*(r^2*tan(psi_cte) - lambda1*r - lambda2*tan(psi_cte) - n*(sigma/phi)/(cos(psi_cte)));
    end
end



%distance to target
r_dist = (WP(1, current)-eta(1))^2 + (WP(2, current)-eta(2))^2;

if last ~= 1
    
    if current == 1
        x = psi_d;
    else
        x = tau6;
    end
    if r_dist <= r0^2
        if size(WP, 2) ~= current
            current = current +1
        else
            last = 1;
            disp('last waypoint')
        end
    end
else
    x = eta(6);
end
