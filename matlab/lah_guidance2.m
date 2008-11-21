%This function takes the current state as input, including position,
%velocity, a waypoint vector must be supplied. To
%calculate the LOS angle.
function y = lah_guidance(u)


eta = u(1:6);
nu = u(7:12);
p = u(13:15);
heading_p = u(16);
heading_m = u(17);

lookahead = 4;




xp = heading_p;

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