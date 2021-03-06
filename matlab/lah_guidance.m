%This function takes the current state as input, including position,
%velocity, a waypoint vector must be supplied. To
%calculate the LOS angle.
function y = lah_guidance(u)

global WP

eta = u(1:6);
nu = u(7:12);
p = u(13:21);
heading = u(22);

persistent current

if isempty(current)
    current = 2;
    WP = [eta(1:2), WP];
end



lookahead = 5;

if current == 2
    r2 = sqrt((WP(1,current)-eta(1))^2 + (WP(2,current)-eta(2))^2);
    y = atan2(WP(2, current)- eta(2), WP(1, current) - eta(1));
    if r2 <= 10
        current = current +1
    end
else

xp = heading;

if abs(xp-eta(6)) > pi/2
    xp = -xp;
end

% 
% if xp < -pi/2 || xp > pi/2 
%     xp = -xp;
% end

%cross-track error
% 
% if abs(xp) < 0.01;
%     xp = heading_p;
% end



e = -(eta(1) - p(1))*sin(eta(6)) + (eta(2) - p(2))*cos(eta(6));

xr = atan2(-e, lookahead);

%distance to target

% r_dot = (WP(1, current)-eta(1))*(cos(eta(6))*nu(1)-sin(eta(6))*nu(2)) + (WP(2, current)-eta(2))*(sin(eta(6))*nu(1)+cos(eta(6))*nu(2));

   psi_d = xp + xr;
    
    %calculate sideslip angle
%     beta = atan2(nu(2), nu(1));

    %actual heading command
    psi = psi_d;% + beta;

y = psi;
end
end