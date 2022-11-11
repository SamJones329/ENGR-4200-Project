# ENGR-4200-Project
ENGR 4200 Project for improving localization of BlueROV2.

## Progress
- [x] Implement base particle filter implementation for 2D in C++
- [/] Extend PF to 3D
- [] Implement parent node which can accept any localization algorithm based on Bayesian Filtering, specifically any Kalman Filter variant
- [x] Implement testing version of parent node that uses turtlebot bag files
- [] Ascertain proper uncertainties for testing implementation and for BlueROV
- [] Develop better weighting scheme for particle filter 
- [] Test particle filter on BlueROV2
- [] Ensure are using random number generator correctly (see std::random_device)
- [] Create EKF class
- [] Set up command line/launch file arguments or params