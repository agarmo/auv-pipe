%This function takes the current state as input, including position,
%velocity, a waypoint vector must be supplied. To
%calculate the LOS angle.
function y = los_calc2(u)

global WP %global waypoint vector
persistent current last %current WP

eta = u(1:6);
nu = u(7:12);

r0 = 6;
nlpp = 15;


if isempty(current)
    current = 2;
    WP = [eta(1:2), WP];
end
if isempty(last)
    last = 0;
end

x = eta(1);
y = eta(2);

activeWpt = current;

%calculate distance to waypoint
r = sqrt((WP(1, activeWpt) - x)^2 + (WP(2, activeWpt) - y)^2);

    dx = WP(1, activeWpt) - WP(1, activeWpt-1);
    dy = WP(2, activeWpt) - WP(2, activeWpt-1);

if dx ~= 0
    d = dy/dx;
else
    d = 0;
end


e = WP(1, activeWpt-1);
f = WP(2, activeWpt-1);
g = f - d*e;

a = 1 +d^2;
b = 2*(d*g - d*y - x);
c = x^2 + y^2 + g^2 - r^2 - 2*g*y;

if dx > 0 
    xlos = (-b + sqrt(b^2 - 4*a*c))/(2*a);
    ylos = (dy/dx)*(xlos - WP(1, activeWpt-1)) + WP(2, activeWpt-1);
elseif dx < 0
    xlos = (-b - sqrt(b^2 - 4*a*c))/(2*a);
    ylos = (dy/dx)*(xlos - WP(1, activeWpt-1)) + WP(2, activeWpt-1);
else
    xlos = WP(1, activeWpt-1);
    if dy > 0
        ylos = y +nlpp;
    else
        ylos = y - nlpp;
    end
end




%distance to target

% r_dot = (WP(1, current)-eta(1))*(cos(eta(6))*nu(1)-sin(eta(6))*nu(2)) + (WP(2, current)-eta(2))*(sin(eta(6))*nu(1)+cos(eta(6))*nu(2));


%check if distance to wp is less than r0
if r <= r0 %|| r_dot < 0
    if size(WP, 2) ~= current 
        current = current +1
    else
        last = 1;
        disp('last waypoint')
    end
end

%compute line of sight angle
if last ~= 1
    xp = atan2(WP(2, current) - WP(2, current-1), WP(1, current)-WP(1,current-1));

   e = (eta(1) - WP(1,current-1))*cos(xp) - (eta(2) - WP(2, current-1))*sin(xp);
%     psi_d = atan2((ylos-eta(2)),(xlos-eta(1)));
    psi_d = atan2(ylos-eta(2), xlos - eta(1));
    
    %calculate sideslip angle
    beta = atan2(nu(2), nu(1));

    %actual heading command
    psi = psi_d + beta;
else
    psi = 0;
end

y = psi;

end