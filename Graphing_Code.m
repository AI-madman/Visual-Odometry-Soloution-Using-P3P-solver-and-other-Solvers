% %% It is the objective of this file to caculate the error and plot the results.
% %  This is done as nabil wanted to see the graph of the error in the Z
% %  axis, for teh BA based methord. 
% clc
% Postions08=load('Postions08');
% Postions03=load('Postions03');
% 
% ztrue08=Postions08.Postion.true.z(1:4070);
% zest08=Postions08.Postion.estimate.z(1:4070);
% 
% diff08=ztrue08-zest08;
% nonNegDiff08=abs(diff08);
% 
% ztrue03=Postions03.Postion.true.z(1:800);
% zest03=Postions03.Postion.estimate.z(1:800);
% 
% diff03=ztrue03-zest03;
% nonNegDiff03=abs(diff03);
% 
% subplot(2,1,1)
% plot(nonNegDiff03*4);
% title('Error plot of the Z-direction of Sequance 03');
% xlabel('Stereo Pair Index');
% % ylabel('Error in the Z axis');
% % 
% % subplot(2,1,2)
% % plot(nonNegDiff08*2.5);
% % title('Error plot of the Z-direction of Sequance 08');
% % xlabel('Stereo Pair Index');
% % ylabel('Error in the Z axis');
% 
subplot(1,3,1);
hold on;
% plot(abs(Postion.true.x(1:4070)-(-1*Postion.estimate.x(1:4070))));
% plot(Postion.true.x(1:4070)-(-1*Postion.estimate.x(1:4070)));
% legend ('Undirectional Error','Directional Error');
% xlabel('Index of Stereo Frames');
% ylabel('Error Measured in Meters');
% title('X Axis');
% 
% subplot(1,3,2);
% hold on;
% % plot(abs(Postion.true.y(1:4070)-Postion.estimate.y(1:4070)));
% plot(Postion.true.y(1:4070)-(-1*Postion.estimate.y(1:4070)));
% legend ('Undirectional Error','Directional Error');
% xlabel('Index of Stereo Frames');
% ylabel('Error Measured in Meters');
% title('Y Axis');
% 
% subplot(1,3,3);
% hold on;
% % plot(abs(Postion.true.z(1:4070)-Postion.estimate.z(1:4070)));
% plot(Postion.true.z(1:4070)-(Postion.estimate.z(1:4070)));
% legend ('Undirectional Error','Directional Error');
% xlabel('Index of Stereo Frames');
% ylabel('Error Measured in Meters');
% title('Z Axis');
% 
%     trueX=Postion.true.x(1:4070);
% %     EstimateX=Postion.estimate.x(1:4070);
% %     diffrancex=Postion.true.x(1:4070)-Postion.estimate.x(1:4070);
% %     t=1:4070;
% %     disp('t              trueX            EstimateX                diffrancex')
% %     disp( [t',trueX',EstimateX',diffrancex'])               
% 
% 

subplot(1,1,1)
scatter3(Postion.true.x(1:4070),Postion.true.y(1:4070),Postion.true.z(1:4070));
hold on;
scatter3(-Postion.estimate.Odometry.x(1:4070),Postion.estimate.Odometry.y(1:4070),Postion.estimate.Odometry.z(1:4070));
legend('Ground Truth Pose', 'Estimated Pose')
xlabel('X Axis');
ylabel('Y Axis');
zlabel('Z Axis');
title('BA Visual Odometry Trajecory Plot');