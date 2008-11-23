close all;


figure(1)
plot(Translation.signals(1,2).values(:,2), Translation.signals(1,1).values(:,2))
hold on
plot(WP(2,:), WP(1,:), 'r*');
plot(pipeline(:,2), pipeline(:,1), 'r--');
% plot(Camsim_output(:,5), Camsim_output(:,4), 'b*');


% plot(eta_beta.signals.values(:,2), eta_beta.signals.values(:,1), 'r')

legend('NE trajectory of AUV', 'LOS Guidance Waypoints', 'Pipeline Trajectory');
title('NE position of AUV movement along waypoints with LOS guidance.');
xlabel('East [m]');
ylabel('North [m]')

hold off
% 

figure(2)
subplot(3, 1, 1)
plot(NE_Veloc.time, NE_Veloc.signals(1,1).values)
xlabel('Time[s]');
ylabel('Surge Speed [m/s]')
legend('Surge Speed');
grid on

subplot(3, 1, 2)
plot(Translation.time, Translation.signals(1,3).values)
xlabel('Time [s]');
ylabel('Down [m]');
title('Depth of AUV versus Reference');
legend('Depth Reference', 'AUV depth');
grid on

subplot(3,1, 3)
plot(Attitude.time, Attitude.signals(1,3).values)
xlabel('Time [s]');
ylabel('Heading [degrees]');
title('Heading reference versus Heading');
legend('\psi_d', '\psi');
grid on


% figure(3)
% subplot(3,1,1)
% plot(Pipeline_pred.time, Pipeline_pred.signals.values(:,1), Pipeline_pred.time, Pipeline_malt.signals.values(:,1));
% title('Pipeline predicted vs Measured')
% subplot(3,1,2)
% plot(Pipeline_pred.time, Pipeline_pred.signals.values(:,2), Pipeline_pred.time, Pipeline_malt.signals.values(:,2));
% 
% subplot(3,1,3)
% plot(Pipeline_pred.time, Pipeline_pred.signals.values(:,3), Pipeline_pred.time, Pipeline_malt.signals.values(:,3));
% 
% figure(4)
% subplot(3,1, 1)
% plot(Pipeline_pred.time, Pipeline_pred.signals.values-Pipeline_malt.signals.values(:,1:3));
% title('Difference between measured and predicted');
% subplot(3, 1,2)
% plot(Pipeline_pred.time, Pipeline_pred.signals.values-Pipeline_malt.signals.values(:,4:6));
% subplot(3, 1,3)
% plot(Pipeline_pred.time, Pipeline_pred.signals.values-Pipeline_malt.signals.values(:,7:9));
% 
% dir = [];
% for i = 1:size(Pipeline_malt.signals.values(:,1))
%     dir = [dir; atan2(Pipeline_malt.signals.values(i,2)-Pipeline_malt.signals.values(i,8), Pipeline_malt.signals.values(i,1)-Pipeline_malt.signals.values(i,7))];
% end


str = size(Attitude.signals(1,3).values, 1);
dec = 200;
% str = ceil(size(Attitude.signals(1,1).values(1,:))/40);
len = ceil(str/dec);
% [u,v] = pol2cart(dir, ones(size(dir)));
% [headx, heady] = pol2cart((pi/180).*Attitude.signals(1,3).values(1:40:size(dir),2), 1.*ones(str));
% [desirx, desiry] = pol2cart((pi/180).*Attitude.signals(1,3).values(1:40:size(dir),1), 1.*ones(str));
% [u,v] = pol2cart(dir, ones(size(dir)));
[headx, heady] = pol2cart((pi/180).*Attitude.signals(1,3).values(1:dec:str,2), 1.*ones(len,1));
[desirx, desiry] = pol2cart((pi/180).*Attitude.signals(1,3).values(1:dec:str,1), 1.*ones(len,1));

figure(5)
% plot(Pipeline_pred.signals.values(:,2), Pipeline_pred.signals.values(:,1), '--b', Pipeline_malt.signals.values(:,2), Pipeline_malt.signals.values(:,1), '.r', Pipeline_malt.signals.values(:,5), Pipeline_malt.signals.values(:,4),'.b', Pipeline_malt.signals.values(:,8), Pipeline_malt.signals.values(:,7), '.k',  Translation.signals(1,2).values(:,2), Translation.signals(1,1).values(:,2), 'g');
plot(P_pred.signals.values(:,2), P_pred.signals.values(:,1), '--b', Translation.signals(1,2).values(:,2), Translation.signals(1,1).values(:,2), 'r');
hold on
% plot(P_post.signals.values(:,2), P_post.signals.values(:,1), '--k')
plot(pipeline(1:300,2), pipeline(1:300,1), '-k', pipeline(500:1400,2), pipeline(500:1400,1), '-k', 'LineWidth', 3);
plot(WP(2,1:2), WP(1,1:2), '*m');
% quiver(Pipeline_malt.signals.values(:,5), Pipeline_malt.signals.values(:,4), v, u, 0.1)
% quiver(Translation.signals(1,2).values(1:40:size(dir),2), Translation.signals(1,1).values(1:40:size(dir),2), heady, headx, 0.2, 'k');
% quiver(Translation.signals(1,2).values(1:40:size(dir),2), Translation.signals(1,1).values(1:40:size(dir),2), desiry, desirx, 0.2, 'y');

quiver(Translation.signals(1,2).values(1:dec:str,2), Translation.signals(1,1).values(1:dec:str ,2), heady, headx, 0.2, 'k');
quiver(Translation.signals(1,2).values(1:dec:str,2), Translation.signals(1,1).values(1:dec:str ,2), desiry, desirx, 0.2, 'b');
 
hold off
% legend('Pipeline Predicted', 'Pipeline Point Stern', 'Pipeline Point Center', 'Pipeline Point Aft', 'AUV trajectory', 'actual pipeline', 'Waypoints', 'Measured Pipeline Direction', 'AUV Heading', 'AUV Desired Heading');

legend('Pipeline Predicted','AUV trajectory', 'actual pipeline','', 'Waypoints', 'AUV Heading', 'AUV Desired Heading');
xlabel('East [m]')
ylabel('North [m]')

figure(6)
subplot(3,1,1)
plot(Cam_output.time, Cam_output.signals.values(:,1:2));
title('Camera output');
subplot(3,1,2)
plot(Cam_output.time, Cam_output.signals.values(:,3:4));
subplot(3,1,3)
plot(Cam_output.time, Cam_output.signals.values(:,5:6));


% figure(7)
% plot(heading.time, (180/pi).*heading.signals.values(:,1), heading.time, (180/pi).*heading.signals.values(:,2))
% title('Heading predicted versus measured');


figure(8)
plot(P_pred.signals.values(:,2), P_pred.signals.values(:,1), 'b', P_post.signals.values(:,5), P_post.signals.values(:,4), 'k');
hold on
plot(pipeline(:,2), pipeline(:,1), '--r');
legend('Pipeline predicted', 'Pipeline updated', 'Actual Pipeline')
hold off


figure(9)
plot(P_pred.time, P_pred.signals.values(:,2)-P_post.signals.values(:,5), P_pred.time, P_pred.signals.values(:,1)-P_post.signals.values(:,4));
grid on

legend('Difference from Predicted and updated, y-direction', 'Difference from Predicted and updated, x-direction')

[pitchx, pitchy, pitchz] = 	sph2cart((pi/180).*Attitude.signals(1,3).values(1:dec:str,2), (pi/180).*Attitude.signals(1,2).values(1:dec:str,2), 1.*ones(len,1));
[dpitchx, dpitchy, dpitchz] = 	sph2cart((pi/180).*Attitude.signals(1,3).values(1:dec:str,1), (pi/180).*Attitude.signals(1,2).values(1:dec:str,1), 1.*ones(len,1));



figure(10)
plot3(Translation.signals(1,2).values(:,2), Translation.signals(1,1).values(:,2), -Translation.signals(1,3).values(:,2))
hold on
quiver3(Translation.signals(1,2).values(1:dec:str,2), Translation.signals(1,1).values(1:dec:str,2), -Translation.signals(1,3).values(1:dec:str,2),pitchy, pitchx, pitchz , 0.2);
quiver3(Translation.signals(1,2).values(1:dec:str,2), Translation.signals(1,1).values(1:dec:str,2), -Translation.signals(1,3).values(1:dec:str,2),dpitchy, dpitchx, dpitchz , 0.2, 'y');
plot3(P_post.signals.values(1:20:end,2), P_post.signals.values(1:20:end,1), -P_post.signals.values(1:20:end,3), '.k')
plot3(pipeline(1:3501,2), pipeline(1:3501,1), -pipeline(1:3501,3));
hold off;
xlabel('East [m]');
ylabel('North [m]');
zlabel('-Down [m]')
grid on

% 
% figure(3)
% plot3(Translation.signals(1,2).values(:,2), Translation.signals(1,1).values(:,2), -Translation.signals(1,3).values(:,2));
% xlabel('East [m]');
% ylabel('North [m]');
% zlabel('Down [m]');
% grid on;
% hold on
% plot3(pipeline(:,2), pipeline(:,1), -pipeline(:,3), 'r--')
% % legend('AUV Trajectory', 'Pipeline Trajectory');
% hold off;
% 
% figure(4)
% plot3(eta.signals.values(:,2), eta.signals.values(:,1), -eta.signals.values(:,3));
% xlabel('East [m]');
% ylabel('North [m]');
% zlabel('Down [m]');
% grid on;
% hold on
% plot3(Camsim_output(:,5), Camsim_output(:,4), -Translation_ref.signals(1,3).values(:,1)-bottom, 'r.')
% % legend('AUV Trajectory', 'Observed pipeline trajectory from camera');
% hold off;
% 
