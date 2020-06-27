clc;
clear;
close all;

%% Execute the configuration file to read parameters for data paths
addpath(genpath(pwd));
configFile;
runningError=0;

%% Starting parallel pooling (requires Parallel Processing Toolbox)
% This section takes a while to load for the first time
% To shutdown, run: delete(gcp('nocreate'));
% if (isempty(gcp) && data_params.use_multithreads)
%     parpool();
% end

if (isempty(gcp) && Switch.ParrallProcessing)
    parpool();
end

%% Read directories containing images
img_files1 = dir(strcat(data_params.path1,'*.png'));
img_files2 = dir(strcat(data_params.path2,'*.png'));
num_of_images = length(img_files1);

%% Read ground truth file if flag is true
if data_params.show_gt_flag
  ground_truth = load(data_params.gt_file);
  gt_x_max = max(ground_truth(:, end - 8));
  gt_x_min = min(ground_truth(:, end - 8));
  gt_z_max = max(ground_truth(:, end));
  gt_z_min = min(ground_truth(:, end));
end

%% Initialize variables for odometry
pos = [0;0;0];
Rpos = eye(3);

%% Start Algorithm
start = 0;
for t = 1:1:num_of_images-1
    %% Read images for time instant t
    I2_l = imread([img_files1(t+1).folder, '/', img_files1(t).name]);
    I2_r = imread([img_files2(t+1).folder, '/', img_files2(t).name]);
    if Switch.UndistortImage==1
        I2_l=undistortImage(I2_l,CameraParams1);
        I2_r=undistortImage(I2_r,CameraParams2);
    end
    fprintf('Frame: %i\n', t);
    if Switch.ImageBlacking==1
        BlackedRows=floor(size(I2_l,1)*ImageBlacking);
        BlackedColoumns=size(I2_l,2);
        I2_l(1:BlackedRows,1:BlackedColoumns)=zeros(BlackedRows,BlackedColoumns);
        I2_r(1:BlackedRows,1:BlackedColoumns)=zeros(BlackedRows,BlackedColoumns);
    end

    %% Bootstraping for initialization
    if (start == 0)
        if Switch.BundleAdjustment==1
        [Points1, Points2]=computeStereoFeatures(I2_l,I2_r,options);
        Algo='Tradtional feature based Bundle Adjustment';
        else
        vo_previous.pts1_l = computeFeatures(I2_l, vo_params.feature);
        vo_previous.pts1_r = computeFeatures(I2_r, vo_params.feature);
        Algo='SOFT based Bundle Adjustment with Circular Matching';
        end
        start = 1;
        I1_l = I2_l;
        I1_r = I2_r;
        T = reshape(ground_truth(t, :), 4, 3)';
        pos_gt = T(:, 4);
        Results.Odometry.GroundX=pos_gt(1);
        Results.Odometry.GroundY=pos_gt(2);
        Results.Odometry.GroundZ=pos_gt(3);
        fprintf('\n---------------------------------\n');
        disp(['Chossen Methord: ' Algo])
        continue;
    end

    %% Implement SOFT for time instant t+1
    if Switch.BundleAdjustment==1
        [R, tr] =BundleAdjustmentAmar(Points1,Points2,P1,P2,I1_l,I2_l,I2_r,Switch,absolutePose,IntrinsicMatrix1);
        [Points1,Points2] = computeStereoFeatures(I2_l,I2_r,options);
    else
    [R, tr, vo_previous] = visualSOFT(t, I1_l, I2_l, I1_r, I2_r, P1, P2, vo_params, vo_previous,Switch);
    end
    %% Estimated pose relative to global frame at t = 0
    pos = pos + Rpos * tr';
    Rpos = R * Rpos;

    %% Prepare frames for next iteration
    I1_l = I2_l;
    I1_r = I2_r;

    %% Plot the odometry transformed data
%     subplot(2, 2, [2, 4]);

        
      T = reshape(ground_truth(t, :), 4, 3)';
      pos_gt = T(:, 4);
    if Switch.ShowScatter==1
    if Switch.ShowGT==1
    %  axis([gt_x_min gt_x_max gt_z_min gt_z_max])
    axis([-500 500 -200 200])
    scatter(pos_gt(1), pos_gt(3), 'r');
    hold on;
    end
    scatter( - pos(1), pos(3), 'b');
    title(sprintf('Odometry plot at frame %d', t))
    xlabel('x-axis (in meters)');
    ylabel('Z-axis (in meters)');

    if Switch.ShowGT==1
        legend('Ground Truth Pose', 'Estimated Pose')
    else
        legend('Estimated Pose')
    end
    end
    
    if Switch.ShowScatter3==1
    if Switch.ShowGT==1
    %  axis([gt_x_min gt_x_max gt_z_min gt_z_max])
    axis([-500 500 -200 200])
    scatter3(pos_gt(1),pos_gt(2), pos_gt(3), 'r');
    hold on;
    end
    scatter3( - pos(1),pos(2), pos(3), 'b');
    title(sprintf('Odometry plot at frame %d', t))
    xlabel('x-axis (in meters)');
    ylabel('Z-axis (in meters)');

    if Switch.ShowGT==1
        legend('Ground Truth Pose', 'Estimated Pose')
    else
        legend('Estimated Pose')
    end
    end
    %% Record Postion of agent at each time step
    Postion.true.x(t)=pos_gt(1);
    Postion.true.y(t)=pos_gt(2);
    Postion.true.z(t)=pos_gt(3);
    Postion.estimate.Odometry.x(t)=pos(1);
    Postion.estimate.Odometry.y(t)=pos(2);
    Postion.estimate.Odometry.z(t)=pos(3);  
    Postion.transtion.Odometry.rotation(:,:,t)=R;
%     Postion.transtion.Odometry.translation(t)=tr;
    %% Caculate distance propagated
    if Switch.CaculateDistancePropagated==1
    Results.Odometry.Xdistance=Results.Odometry.Xdistance + abs(pos_gt(1)-Results.Odometry.GroundX);
    Results.Odometry.Ydistance=Results.Odometry.Ydistance + abs(pos_gt(2)-Results.Odometry.GroundY);
    Results.Odometry.Zdistance=Results.Odometry.Zdistance + abs(pos_gt(3)-Results.Odometry.GroundZ);
    Results.Odometry.XYZdistance=Results.Odometry.XYZdistance + sqrt(square(pos_gt(1)-Results.Odometry.GroundX)+square(pos_gt(2)-Results.Odometry.GroundY)+square(pos_gt(3)-Results.Odometry.GroundZ));
    end
    %% Caculate Error 
    if Switch.CaculateError==1
    %Euclidan Error
    Results.Odometry.error.EuclideianXY(t)=sqrt(square(pos_gt(1)- pos(1)) +square(pos_gt(2)- pos(2)));
    Results.Odometry.error.Euclideian(t)=sqrt(square(pos_gt(1)- pos(1)) +square(pos_gt(2)- pos(2)) +square(pos_gt(3)- pos(3))) ;
    Results.Odometry.runningError=Results.Odometry.runningError+Results.Odometry.error.Euclideian(t);
    Results.Odometry.error.Avarage(t)=Results.Odometry.runningError/t;
%     if t>2
%         Results.error.Avarage2(i)=(Results.error.Euclideian(i)+Results.error.Euclideian(i-1))/i;
%     end
    % Maximum Error
    if Results.Odometry.error.Euclideian(t)> Results.Odometry.error.Maximum
        Results.Odometry.error.Maximum=Results.Odometry.error.Euclideian(t);
    end
    end
    %% Update Ground Truth 
    Results.Odometry.GroundX=pos_gt(1);
    Results.Odometry.GroundY=pos_gt(2);
    Results.Odometry.GroundZ=pos_gt(3);
    %% Pause to visualize the plot
    if Switch.ViewPlot==1
    pause(ViewPlotTime);
    end
    disp(['PERCENTAGE COMPLETE: ',num2str(100*t/num_of_images)])
    fprintf('\n---------------------------------\n');
end

%% Save workSpace
if Switch.SaveVO==1
save(VOSavePath);
end

%% Find Adjusted Z
if Switch.ImageBlackingAdjusted==1
start = 0;
for t = 1:1:num_of_images-1
    %% Read images for time instant t
    I2_l = imread([img_files1(t+1).folder, '/', img_files1(t).name]);
    I2_r = imread([img_files2(t+1).folder, '/', img_files2(t).name]);
    fprintf('Frame: %i\n', t);
        BlackedRows=floor(size(I2_l,1)*ImageBlackingAdjusted);
        BlackedColoumns=size(I2_l,2);
        I2_l(1:BlackedRows,1:BlackedColoumns)=zeros(BlackedRows,BlackedColoumns);
        I2_r(1:BlackedRows,1:BlackedColoumns)=zeros(BlackedRows,BlackedColoumns);

    %% Bootstraping for initialization
    if (start == 0)
        vo_previous.pts1_l = computeFeatures(I2_l, vo_params.feature);
        vo_previous.pts1_r = computeFeatures(I2_r, vo_params.feature);
        start = 1;
        I1_l = I2_l;
        I1_r = I2_r;
        T = reshape(ground_truth(t, :), 4, 3)';
        pos_gt = T(:, 4);
        Results.Odometry.GroundX=pos_gt(1);
        Results.Odometry.GroundY=pos_gt(2);
        Results.Odometry.GroundZ=pos_gt(3);
        fprintf('\n---------------------------------\n');
        continue;
    end

    %% Implement SOFT for time instant t+1
    [R, tr, vo_previous] = visualSOFT(t, I1_l, I2_l, I1_r, I2_r, P1, P2, vo_params, vo_previous,Switch);

    %% Estimated pose relative to global frame at t = 0
    pos = pos + Rpos * tr';
    Rpos = R * Rpos;
    Postion.estimate.Odometry.zAdjusted(t,1)=pos(3);
    Postion.estimate.Odometry.xAdjusted(t,1)=pos(1);
    Postion.estimate.Odometry.yAdjusted(t,1)=pos(2);
    %% Prepare frames for next iteration
    I1_l = I2_l;
    I1_r = I2_r;
end
end
%% Save workSpace
if Switch.ImageBlackingAdjusted==1
save(VOAdjustedSavePath);
end

%% Error Diagonistics