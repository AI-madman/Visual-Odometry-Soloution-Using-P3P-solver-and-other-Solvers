# Visual-Odometry-Soloution-Using-P3P-solver-and-other-Solvers
Solves the Problem of Pose Estimation on the KITTI Dataset

This project substainly improves the orginal code base by Mayankm96 [1]. 
This is done by adding various Solver metords and not just leaving the final estimation of the SE(3) group element to the P3P algorthims found in Matlab [2].
This new project also allows for various alteratins such as Image Blackening which was devoloped as methord to improve the results gained from [2] as the error along the Z 
axis was orders of maginatude greather than it should have been. Image blacking removes the top portion of the image in such a way as to prevent alterning the dimentions of the image and ensuring the portion of the image places no role in feature detection or in the exxtracted discriptors.

There are a lot of other new features added to this so please feel free to gain an understanding of how they work from the Switches.txt file and employ any combination of them by editing the Swicthes portion of the config.m file. Note that the if a Swich preforms some process that requires the setting of hyperparamters these can be found in the section directly after the Swiches section in the config.m file.

Please enjoy, Fork and Improve.
/





[1] https://github.com/Mayankm96/Stereo-Odometry-SOFT.

[2] https://uk.mathworks.com/help/vision/ref/estimateworldcamerapose.html.
