%This function takes the current state as input, including position,
%velocity, a waypoint vector must be supplied. To
%calculate the LOS angle.
function y = los_calc(u)

global WP %global waypoint vector
persistent current last %current WP

if isempty(current)
    current = 1;
end
if isempty(last)
    last = 0;
end

%r0 = 6; %circle of acceptance Too big!
r0 = 4;

eta = u(1:6);
nu = u(7:12);

%distance to target
r = (WP(1, current)-eta(1))^2 + (WP(2, current)-eta(2))^2;

% r_dot = (WP(1, current)-eta(1))*(cos(eta(6))*nu(1)-sin(eta(6))*nu(2)) + (WP(2, current)-eta(2))*(sin(eta(6))*nu(1)+cos(eta(6))*nu(2))

%check if distance to wp is less than r0
if r <= r0^2 %&& r_dot < 0
    if size(WP, 2) ~= current 
        current = current +1
    else
        last = 1;
        disp('last waypoint')
    end
end

%compute line of sight angle
if last ~= 1
   
    psi_d = atan2(WP(2, current)-eta(2), WP(1, current)-eta(1));
    
    %calculate sideslip angle
    beta = atan2(nu(2), nu(1));

    %actual heading command
    psi = psi_d + beta;
else
    psi = 0;
end

y = psi;

end