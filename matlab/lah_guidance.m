%This function takes the current state as input, including position,
%velocity, a waypoint vector must be supplied. To
%calculate the LOS angle.
function y = lah_guidance(u)


eta = u(1:6);
nu = u(7:12);
p = u(13:18);
heading = u(19);

lookahead = 10;




xp = atan2(p(5)-eta(2),p(4)-eta(1));

%cross-track error

if abs(xp) <= 0.01;
    xp = heading;
end

e = (eta(1) - p(4))*cos(xp) - (eta(2) - p(5))*sin(xp);

xr = atan2(-e, lookahead);

%distance to target

% r_dot = (WP(1, current)-eta(1))*(cos(eta(6))*nu(1)-sin(eta(6))*nu(2)) + (WP(2, current)-eta(2))*(sin(eta(6))*nu(1)+cos(eta(6))*nu(2));

   psi_d = xp + xr;
    
    %calculate sideslip angle
    beta = atan2(nu(2), nu(1));

    %actual heading command
    psi = psi_d + beta;

y = psi;

end