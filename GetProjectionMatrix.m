function [P1,P2]=GetProjectionMatrix(DataSet,rectification)
% This function consrtucts the Projection matrix for the KITTI
% dataset. The dataSet can be segemented into various devisions
% such as City or Residental or Road or Campus.  
% Each subset of the dataset comes with a rectification verient.

% DataSet options:
% 1 -> City
% 2 -> Residential
% 3 -> Road
% 4 -> Campus

% rectification
% 0 -> nonRect
% 1 -> Rect

if DataSet==2
    if rectification==1
        P1=([7.070912e+02 0.000000e+00 6.018873e+02 0.000000e+00; 0.000000e+00 7.070912e+02 1.831104e+02 0.000000e+00; 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00]);
        P2=([7.070912e+02 0.000000e+00 6.018873e+02 -3.798145e+02; 0.000000e+00 7.070912e+02 1.831104e+02 0.000000e+00; 0.000000e+00 0.000000e+00  1.000000e+00 0.000000e+00]);
    end
end