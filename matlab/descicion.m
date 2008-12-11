function y = descicion(u)
    
    global WP pipeline_dir trajectory_generated
    
    %input: Output, eta, Pipeline trajectory,
    eta = u(1:6);
    nu = u(7:12);
    p = u(13:21);
    heading = u(22);
    output = u(23);
    t = u(24);
    
    lookahead = 5; %5
    lookahead_s = 6; %6
    
    persistent mode current current_s detected_pos time_since_contact last_known_pos
    % mode = 2 goto; mode=1 serach; mode = 0 track
    
    if isempty(mode) || isempty(current) || isempty(detected_pos) || isempty(current_s)
        mode = 2; %start in goto mode
        current = 2;
        current_s = 2;
        WP = [eta(1:2), WP];
        detected_pos = [0; 0];
        time_since_contact = 0;
        trajectory_generated=0;
        last_known_pos = [0 ; 0];
    end
    
    
    
    
    switch mode
        
        case 2 %goto mode
            r2 = sqrt((WP(1,current)-eta(1))^2 + (WP(2,current)-eta(2))^2);
            y = atan2(WP(2, current)- eta(2), WP(1, current) - eta(1));
               if r2 <= 5
                  current = current +1
                  
                  if output == 1
                      mode = 0
                      detected_pos = p(4:5)
                      return;
                  else
                      mode = 1
                      return;
                  end
               end

            
        case 1 %search mode Generate trajectory
            if output == 1 % && trajectory_generated ~= 1
                detected_pos = p(4:5)
                y = atan2(detected_pos(2) - eta(2), detected_pos(1) - eta(1))
                mode = 0
                trajectory_generated = 1
                return
            else
               if trajectory_generated == 2
                    [WP, current_s] = generate_WP(last_known_pos, pipeline_dir);
               elseif trajectory_generated == 0
                   [WP, current_s] = generate_initial_search(WP(1:2, 2));
               end
               

               r2_t = sqrt((WP(1,current_s)-eta(1))^2 + (WP(2,current_s)-eta(2))^2);
               
               if r2_t <= 6
                  current_s = current_s + 1
               end
               xi_wp = atan2(WP(2, current_s)-WP(2, current_s-1), WP(1, current_s) - WP(1, current_s-1));
               
               if xi_wp < 0 %fix for not turning all the way around
                   xi_wp = xi_wp + 2*pi;
               end
               
               e = -(eta(1) - WP(1, current_s-1))*sin(xi_wp) + (eta(2) - WP(2, current_s-1))*cos(xi_wp);
               y = xi_wp + atan2(-e, lookahead_s);
               
               if output == 1 && (trajectory_generated == 1 || trajectory_generated == 3)
                   mode = 0;
                   return;
               elseif output == 1
                   mode = 0
                   detected_pos = p(4:5)
                   trajectory_generated = 1
                   return;
               end
               
            end

        case 0 %track mode
            xp = heading;
            if output == 0
                 %lost measurement
                 if time_since_contact == 0
                     time_since_contact = t
                     last_known_pos = p(4:5);
                 end
            end
            
            if abs(xp-eta(6)) > pi/2
                xp = -xp;
            end

            e = -(eta(1) - p(1))*sin(eta(6)) + (eta(2) - p(2))*cos(eta(6));

            xr = atan2(-e, lookahead);

            psi_d = xp + xr;
            y = psi_d;
            
            if t - time_since_contact == 10 %60 scn3 %25 scn 1 og 2 og 4
                mode = 1 %search mode
                time_since_contact = 0;
                trajectory_generated = 2;
                return;
            end 

    end
    last_psi_d = y;
end


function [waypoint, current] = generate_WP(eta_t, pipeline_dir)
    
    global trajectory_generated
    
    %position from origo of NED
    r1 = 80;
    r2 = 100;
    r3 = 200;
     
    %Search boundaries
    theta_min = pipeline_dir - pi/20;
    theta_max = pipeline_dir + pi/20;
    
    waypoint = [eta_t(1), eta_t(1)+r1*cos(theta_max), eta_t(1)+r2*cos(theta_min), eta_t(1)+r3*cos(theta_max);
                eta_t(2), eta_t(2)+r1*sin(theta_max), eta_t(2)+r2*sin(theta_min), eta_t(2)+r3*sin(theta_max)]
    
    trajectory_generated = 1;
    current = 2;
    
end


function [waypoint, current] = generate_initial_search(eta_0)
    %generate spiral pattern around initial condition.
    global trajectory_generated
    theta = 0:0.1:8*pi; %4 omdreininger
    b = 30;
    r = theta + b*theta; %spiral i polarkoordinater
    [x, y] = pol2cart(theta, r);
    waypoint = [eta_0(1); eta_0(2)];
    for i = 1:15:size(x, 2)
         waypoint = [waypoint, [eta_0(1)+x(i); eta_0(2)+y(i)]]
    end
    trajectory_generated = 1;
    current = 2;
    
end

function [waypoint, current] = generate_turn_trajectory(detected_pos, eta_t, pipeline_dir)
    %takes in detected point, pipeline direction and wp vector
    %outputs new wp vector with new wps
    
    global trajectory_generated
    psi = eta_t(6);
    
    r1 = 20;
    r2 = 50;
    r3 = 60;
    r4 = 60;
    if psi > pi/2
        theta1 = psi + pi/6;
        theta2 = psi + pi/3;
        theta3 = psi + 2*pi/5;
        theta4 = psi + 2*pi/3;
    else
        theta1 = psi - pi/6;
        theta2 = psi - pi/3;
        theta3 = psi - 2*pi/5;
        theta4 = psi - 2*pi/3;
    end
    
     if (psi > pipeline_dir + pi/8) || psi < (pipeline_dir - pi/8)
        waypoint = [eta_t(1), eta_t(1)+r1*cos(theta1), eta_t(1)+r2*cos(theta2), eta_t(1)+r3*cos(theta3), detected_pos(1), detected_pos(1)+r2*cos(pipeline_dir);
                    eta_t(2), eta_t(2)+r1*sin(theta1), eta_t(2)+r2*sin(theta2), eta_t(2)+r3*sin(theta3), detected_pos(2), detected_pos(2)+r2*sin(pipeline_dir)]
     else
         waypoint = [eta_t(1), detected_pos(1);
                     eta_t(2), detected_pos(2)];
     end
    
    trajectory_generated = 3
    current = 2;
    
    
end









