# mail-delivery-bot
Final course project for ROB301

The goal for this project was to program a simple turtle bot to:
1. Follow a line on the ground.
1. Use computer vision to determine where the line is or what colour square the robot is on.
1. Perform SLAM to determine if the robot is on the right square (and deliver the package if so).


This project involved the use of:
- Control systems and dynamics to control the robot to perform specific actions.
- ROS to physically control the robot system.
- Basic computer vision processing to gather data on the environment from the onboard camera.
- Bayesian localization to convert data from the camera into a map of the track, qualified with uncertainty.

