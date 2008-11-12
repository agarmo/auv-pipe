function [x] = camsim3(u)

%% Initialization
global pipeline focus

t = u(32);
noise = u(33);
variance = u(34);

DimX = 400;
DimY = 400;



persistent output isOutput



if mod(t, 1) == 0 %Every 1 second there is a new sample
    eta = u(1:6);
    bottom = u(13);
%% Rotation matrix    
    Rot = [u(14:16)';
             u(17:19)';
             u(20:22)'];

    Trans = [u(23:25)';
           u(26:28)';
           u(29:31)'];


    %% Check if pipeline is inside FOV

    % determine fov
    % eta_b = inv(J)*eta; %transform to body coorodinates

    z = bottom - eta(3); %altitude

    fov = z*tand(60/2);

    eta_b = inv(Rot(1:2, 1:2))*eta(1:2);

    % fov_ned = Rot(1:2,1:2)*[fov, fov]';
    % 
    % if eta(1) < 0
    %     x_max = eta(1) - fov_ned(1);
    %     x_min = eta(1) + fov_ned(1);
    % else
    %     x_max = eta(1) + fov_ned(1);
    %     x_min = eta(1) - fov_ned(1);
    % end
    % 
    % if eta(2) < 0
    %     y_max = eta(2) - fov_ned(2);
    %     y_min = eta(2) + fov_ned(2);
    % else
    %     y_max = eta(2) + fov_ned(2);
    %     y_min = eta(2) - fov_ned(2);
    % end

        x_max = eta_b(1) + fov;
        x_min = eta_b(1) - fov;
        y_max = eta_b(2) + fov;
        y_min = eta_b(2) - fov;


%     max = Rot(1:2,1:2)*[x_max, y_max]';
%     max2 = Rot(1:2, 1:2)*[x_max, y_min]';
%     min = Rot(1:2,1:2)*[x_min, y_min]';
%     min2 = Rot(1:2, 1:2)*[x_min, y_max]';
% 
%     x_max = max(1);
%     y_max = max(2);
%     x_min = min(1);
%     y_min = min(2);

%         if x_max < x_min
%             x_max2 = x_max;
%             x_max = x_min;
%             x_min = x_max2;
%         elseif x_max == x_min
%             x_max = x_max + fov;
%             x_min = x_max - fov;
%         end
%         if y_max < y_min
%             y_max2 = y_max;
%             y_max = y_min;
%             y_min = y_max2;
%         elseif y_max == y_min
%             y_max = y_max + fov;
%             y_min = y_max - fov;
%         end
%     [max', min', min2', max2']'

    pipeline_b = zeros(size(pipeline,1), 3);


    % temp_max = J*[x_max; y_max; eta_b(3:6)];
    % temp_min = J*[x_min; y_min; eta_b(3:6)];
    % % 
    % % temp_max - temp_min
    % 
    % x_max = temp_max(1);
    % x_min = temp_min(1);
    % y_max = temp_max(2);
    % y_min = temp_min(2);

    pipeline_inside = [];
    for i = 1:size(pipeline,1)
        if z <= 6
            pipeline_b(i,:)= (inv(Rot)*pipeline(i,:)')';
            if (pipeline_b(i,1) <= x_max) && (pipeline_b(i,1) >= x_min) %inside x direction
                if (pipeline_b(i,2) <= y_max) && (pipeline_b(i, 2) >= y_min) %inside y direction
                    %
%                                     x_max
%                                     x_min
%                                     y_max
%                                     y_min
%                     pipeline_b(i,:)
                    pipeline_inside = [pipeline_inside; pipeline_b(i,:)]; %#ok<AGROW>

                    %         disp('ingen punkter')
                end
            end
        else
            disp('for mørkt');
        end
    end

    if isempty(pipeline_inside)
    %     disp('ingen punkter i omr�det');
        P = zeros(6,1);
        isOutput = 0;
    else
        % convert pipeline_inside into screen cooridnates
        
        %perspektivligningene
        perspektiv = [(1/z)*focus 0; 0 (1/z)*focus];
        
        temp = size(pipeline_inside,1);

%         if temp ~= 1
%         
%             [Cm,Im] = max(pipeline_inside);
%         
%             [Cmin, Imin] = min(pipeline_inside);
%         else
%             Cm = pipeline_inside;
%             Cmin = pipeline_inside;
%         end
        
        B = sort(pipeline_inside, 'descend');
        
        P = [perspektiv*(B(1,1:2)'-eta_b(1:2));
             perspektiv*(B(ceil(size(B,1)/2), 1:2)'-eta_b(1:2));
             perspektiv*(B(size(B,1),1:2)'-eta_b(1:2))];
         isOutput = 1;
    end

    if noise == 1
    
        noise = variance.*randn(6,1);
    
        output = P + noise;
    else
        output = P;
    end
    
    x = [output; isOutput];
else
    if isempty(output)
        output = zeros(6,1);
        x = [output; isOutput];
    else
        x = [output; isOutput]; 
    end
end