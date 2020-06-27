%% ------------------------------------------------------------------------------
% Configuration File for Visual Odometry Algorithm
%% -------------------------------------------------------------------------------
addpath(genpath('/home/amar/Documents/MATLAB/BASOFTNOCHANGE/Stereo-Odometry-SOFT/code'));

Sequance='07';

% Path to the directories containing images
data_params.path1 = '/home/amar/Desktop/KITTI VO/07/image_0/';
data_params.path2 = '/home/amar/Desktop/KITTI VO/07/image_1/';

S1=imageSet(data_params.path1);
S2=imageSet(data_params.path2);

% Path to calibration text file
data_params.calib_file = '../data/kitti/07/calib.txt';

% Path to groundtruth poses. Set flag to 1 to plot groundtruth as well
data_params.gt_file = '/home/amar/Desktop/KITTI VO/poses/07.txt';
data_params.show_gt_flag = 1;
 
%% load in Ground Thruth
ground_truth = load(data_params.gt_file);
GTAray=zeros(3,1);
for t=1:1:S1.Count
      T = reshape(ground_truth(t, :), 4, 3)';
      pos_gt = T(:, 4); 
      GTAray(:,:,t)=pos_gt;
end
load('RigidBodyTransforms.mat');
%% Read camera parameters
%[P1, P2] = createCamProjectionMatrices(cam_params);
[P1,P2]=GetProjectionMatrix(2,1);
%% Switches 
% Use parallel threads (requires Parallel Processing Toolbox)
data_params.use_multithreads = 0;  
Switch.ParrallProcessing=0;
Switch.EssetialMatrix=0;                         
Switch.ShowGT=1;
Switch.Bucketing=0;
Switch.ShowScatter=0;
Switch.ShowScatter3=0;
Switch.BundleAdjustment=1;                       
Switch.ImageBlacking=0;    
Switch.ViewPlot=0;
Switch.CaculateDistancePropagated=1;
Switch.CaculateError=1;
Switch.SaveVO=0;
Switch.SaveVOAdjusted=0;
Switch.ImageBlackingAdjusted=0;
Switch.BundleAdjustmentMethord=3;
Switch.UndistortImage=0;
%% Switch Paramters
ImageBlacking=0.1;   
ImageBlackingAdjusted=0.1;      
ViewPlotTime=0.0100;
VOSavePath='';
VOAdjustedSavePath='';
options.GoodFeatures2Track=1;
W_C.ROT_W_C=eye(3);
W_C.Trans_W_C=[0 0 0];
absolutePose = rigid3d(W_C.ROT_W_C,W_C.Trans_W_C);
tform = rigid3d;
%% Read directories containing images
img_files1 = dir(strcat(data_params.path1,'*.png'));
img_files2 = dir(strcat(data_params.path2,'*.png'));
num_of_images = length(img_files1);

% Arrays Storing Error Values
Results.Odometry.error.Euclideian=[];
Results.Odometry.error.EuclideianXY=[];
Results.Odometry.error.Avarage=[];
Results.Odometry.error.Avarage2=[];
Results.Odometry.error.Maximum=0;
Results.Odometry.Xdistance=0; % number of meters travled in X direction
Results.Odometry.Ydistance=0; % number of meters travled in Y direction
Results.Odometry.Zdistance=0; % number of meters travled in Z direction
Results.Odometry.XYZdistance=0; % number of meters travled in XYZ direction
Results.Odometry.runningError=0;

Results.EssentialMatrix.error.Euclideian=[];
Results.EssentialMatrix.error.EuclideianXY=[];
Results.EssentialMatrix.error.Avarage=[];
Results.EssentialMatrix.error.Avarage2=[];
Results.EssentialMatrix.error.Maximum=0;
Results.EssentialMatrix.Xdistance=0; % number of meters travled in X direction
Results.EssentialMatrix.Ydistance=0; % number of meters travled in Y direction
Results.EssentialMatrix.Zdistance=0; % number of meters travled in Z direction
Results.EssentialMatrix.XYZdistance=0; % number of meters travled in XYZ direction
Results.EssentialMatrix.runningError=0;

% Results.Filter.error.Euclideian=[];
% Results.Filter.error.EuclideianXY=[];
% Results.Filter.error.Avarage=[];
% Results.Filter.error.Avarage2=[];
% Results.Filter.error.Maximum=0;
% Results.Filter.Xdistance=0; % number of meters travled in X direction
% Results.Filter.Ydistance=0; % number of meters travled in Y direction
% Results.Filter.Zdistance=0; % number of meters travled in Z direction
% Results.Filter.XYZdistance=0; % number of meters travled in XYZ direction
% Results.Filter.bias.accelerometer=zeros(S1.Count);
% Results.Filter.bias.gyroscope=zeros(S1.Count);
% Results.Filter.runningError=0;


%Postion of previos state
Results.Odometry.GroundX=0;
Results.Odometry.GroundY=0;
Results.Odometry.Groundz=0;

% Postions of Agent
Postion.true.x=zeros(S1.Count,1);
Postion.true.y=zeros(S1.Count,1);
Postion.true.y=zeros(S1.Count,1);

Postion.estimate.Odometry.x=zeros(S1.Count,1);
Postion.estimate.Odometry.y=zeros(S1.Count,1);
Postion.estimate.Odometry.z=zeros(S1.Count,1);

Postion.estimate.Odometry.xAdjusted=zeros(S1.Count,1);
Postion.estimate.Odometry.yAdjusted=zeros(S1.Count,1);
Postion.estimate.Odometry.zAdjusted=zeros(S1.Count,1);

% Postion.estimate.Filter.x=zeros(S1.Count);
% Postion.estimate.Filter.y=zeros(S1.Count);
% Postion.estimate.Filter.z=zeros(S1.Count);

Postion.transtion.Odometry.rotation=zeros(3,3);
Postion.transtion.Odometry.translation=zeros(3,1);

% Postion.transtion.Filter.rotation=zeros(3,3);
% Postion.transtion.Filter.translation=zeros(3,1);

% Essential Matrix Relative Transformations
EssentialMatrix.RT.Rotation=zeros(3,3,S1.Count);
EssentialMatrix.RT.Translations=zeros(3,S1.Count);

% Bundle Adjustment World Pose
BundleAdjustment.RT.Rotation=zeros(3,3,S1.Count);
BundleAdjustment.RT.Translations=zeros(3,S1.Count);

%% The calibration parameters of the cameras

% calibration parameters for sequence 2010_03_09_drive_0000
cam_params.fx = 7.188560000000e+02;               % focal length (u-coordinate) in pixels
cam_params.cx = 6.071928000000e+02;               % principal point (u-coordinate) in pixels
cam_params.fy = 7.188560000000e+02;               % focal length (v-coordinate) in pixels
cam_params.cy = 1.852157000000e+02;               % principal point (v-coordinate) in pixels
cam_params.base = 3.861448000000e+02;             % baseline in meters (absolute value)
% cam_params.q_CI =rotMatToQuat(C_c_I);             % 4x1 IMU to the first camera rotation (quaternion)
% cam_params.p_C_I =p_C_I;                          % Postion of the first camrea in IMU frame of refrance
% cam_params.q2_CI =rotMatToQuat(C_c2_I);             % 4x1 IMU to the Seconed camera rotation (quaternion)
% cam_params.p2_C_I =camera.p_C_I + C_c2_I'*(-camera.p_C1_C2);   % Postion of the Seconed camrea in IMU frame of refrance
% cam_params.q_C2C1 = rotMatToQuat(camera.C_C2_C1);   % quaternion rotation between two cameras.
% cam_params.p_C1_C2 = [-5.37*10^-1; 4.822061*10^-3; -1.252488*10^-2]; 
RadialDistortion1=[-3.728755e-01 2.037299e-01 -7.233722e-02];
RadialDistortion2=[-3.644661e-01 1.790019e-01 -5.314062e-02];
TangentialDistortion1=[2.219027e-03 1.383707e-03];
TangentialDistortion2=[1.148107e-03 -6.298563e-04];
Intrinsics1=cameraIntrinsics([P1(1, 1), P1(2,2)], [P1(1, 3), P1(2, 3)],size(read(S1,1)));
Intrinsics2=cameraIntrinsics([P2(1, 1), P2(2,2)], [P2(1, 3), P2(2, 3)],size(read(S2,1)));
IntrinsicMatrix1=[cam_params.fx 0 0; 0 cam_params.fy 0; cam_params.cx cam_params.cy 1];
CameraParams1=cameraParameters('IntrinsicMatrix',Intrinsics1.IntrinsicMatrix,'TangentialDistortion',TangentialDistortion1,'RadialDistortion',RadialDistortion1);
CameraParams2=cameraParameters('IntrinsicMatrix',Intrinsics2.IntrinsicMatrix,'TangentialDistortion',TangentialDistortion2,'RadialDistortion',RadialDistortion2);
StereoParams=stereoParameters(CameraParams1,CameraParams2,eye(3),RigidTransform.Cam_Cam2.Translation');
%% Parameters for Feature Extraction
vo_params.feature.nms_n = 8;                      % non-max-suppression: min. distance between maxima (in pixels)
vo_params.feature.nms_tau = 50;                   % non-max-suppression: interest point peakiness threshold
vo_params.feature.margin = 21;                    % leaving margin for safety while computing features ( >= 25)

%% Parameters for Feature Matching
vo_params.matcher.match_binsize = 50;             % matching bin width/height (affects efficiency only)
vo_params.matcher.match_radius = 200;             % matching radius (du/dv in pixels)
vo_params.matcher.match_disp_tolerance = 1;       % dx tolerance for stereo matches (in pixels)
vo_params.matcher.match_ncc_window = 21;          % window size of the patch for normalized cross-correlation
vo_params.matcher.match_ncc_tolerance = 0.3;      % threshold for normalized cross-correlation

vo_params.matcher.refinement = 2;                 % refinement (0=none,1=pixel,2=subpixel)

%% Paramters for Feature Selection using bucketing
vo_params.bucketing.max_features = 1;             % maximal number of features per bucket
vo_params.bucketing.bucket_width = 50;            % width of bucket
vo_params.bucketing.bucket_height = 50;           % height of bucket

vo_params.bucketing.age_threshold = 10;           % age threshold while feature selection

%% Paramters for motion estimation

vo_params.estim.ransac_iters = 200;              % number of RANSAC iterations
vo_params.estim.inlier_threshold = 2.0;          % fundamental matrix inlier threshold
vo_params.estim.reweighing = 1;                  % lower border weights (more robust to calibration errors)


