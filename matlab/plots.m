close all;

figure(1)
plot(NEpos.signals.values(:,2), NEpos.signals.values(:,1));
hold on
plot(pipeline(:,2), pipeline(:,1), 'g--')
plot(points.signals(1,2).values(:,2), points.signals(1,2).values(:,1), 'r.');
hold off
legend('NE path of AUV', 'Pipeline trajectory', 'Sampled singnal from CAMsim');
xlabel('East [m]');
ylabel('North [m]');
title('North-East position of pipeline, AUV, and observed pipeline'); 