function [Points1,Points2] = computeStereoFeatures(Image1,Image2,options)
%Returns Stereo Corrpondance pairs
%takes in a pair of stereo images and the detection methord and uses 
%them to construct the stereo corrpondance.

if options.GoodFeatures2Track==1
    kp1=detectMinEigenFeatures(Image1);
    kp2=detectMinEigenFeatures(Image2);
    [dis1,valid_points1]=extractHOGFeatures(Image1,kp1);
    [dis2,valid_points2]=extractHOGFeatures(Image2,kp2);
elseif options.Harris==1
    kp1=detectHarrisFeatures(Image1);
    kp2=detectHarrisFeatures(Image2);
    [dis1,valid_points1]=extractHOGFeatures(Image1,kp1);
    [dis2,valid_points2]=extractHOGFeatures(Image2,kp2);
end
indexPairs = matchFeatures(dis1,dis2,'MatchThreshold',0.3,'Unique',true);
Points1 = valid_points1(indexPairs(:,1),:);
Points2 = valid_points2(indexPairs(:,2),:);
disp(['Number of Points in Stereo Corrpondance:' int2str(length(Points1))])

end

