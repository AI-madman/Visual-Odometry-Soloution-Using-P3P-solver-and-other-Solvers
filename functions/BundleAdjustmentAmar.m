function[R, tr]=BundleAdjustmentAmar(pointsLeft,PointsRight,P1,P2,I1_l,I2_l,I2_r,Switch,absolutePose,IntrinsicMatrix1)
%Returns R,T and VO struct.
%Takes in the previous pair of stereo points and all four images.
%triangulates points and does a projection from points2world and
%world2points. Then uses bundle adjutment to solve for R and T.

%% triangulate World Points
worldPoints3D=triangulate(pointsLeft,PointsRight,P1',P2');

%% Track Left Stereo Corrponding Points in time
TrackerLeft=vision.PointTracker('MaxBidirectionalError',0.4,'MaxIterations',50);
initialize(TrackerLeft,pointsLeft.Location,I1_l);
[points,point_validity] = TrackerLeft(I2_l);
disp(['Number of tracked Points:' int2str((size(points,1)))])
%% Update points to match in time
points=points(point_validity,:);
worldPoints3D=worldPoints3D(point_validity,:);
%% Intrinisct Matrix
dims=size(I2_r);
cam1 = cameraIntrinsics([P1(1, 1), P1(2,2)], [P1(1, 3), P1(2, 3)], dims);
%% Rotation (R) and Translation(tr) Estimation by minimizing Reprojection Error
if Switch.BundleAdjustmentMethord==1
    refinedPose = bundleAdjustmentMotion(worldPoints3D,points,absolutePose,cam1);
    R = refinedPose.Rotation;
    tr = refinedPose.Translation;
elseif Switch.BundleAdjustmentMethord==2
    x3d_h=MakePointsHomogenous(worldPoints3D);
    x2d_h=MakePointsHomogenous(points);
    [R,tr]=efficient_pnp(x3d_h,x2d_h,IntrinsicMatrix1);
    tr=tr';
elseif Switch.BundleAdjustmentMethord==3
    [R,tr] = PnPLinear(points,worldPoints3D,IntrinsicMatrix1);
else
[R, tr] = estimateWorldCameraPose(points, worldPoints3D, cam1, 'MaxReprojectionError', 0.4);
end
end

function [points]=MakePointsHomogenous(points)
s=size(points,2);
for i=1:1:size(points,1)
    points(i,s+1)=1;
end
end