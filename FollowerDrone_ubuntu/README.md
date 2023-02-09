# Follower drone
This is the python scripts for the follower drone.  
We have only confirmed it works with Parrot Anafi 4K and Ubuntu 21.04.

## Required libralies

* [Olympe](https://developer.parrot.com/docs/olympe/installation.html)
* OpenCV with CUDA
* [CasADi](https://web.casadi.org/)
* pygame


## Description of each file

### geocalc.py
Class to calculate the distance in GPS coordinates between two points using the Vincenty formulae.

### UndistortMtx.npz
File containing parameters for distortion correction.  
We calculated these parameters using cv2.calibrateCamera(), cv2.getOptimalNewCameraMatrix() and cv2.initUndistortRectifyMap().  
Since these parameters vary from camera to camera, it is recommended to calculate the parameters for your device in advance.

### ARBird_final.py
Script to run BirdViewAR.
Before run this script,
1. Connect your PC to controller via usb - usb type-C cable.
2. Connect your PC to the wifi network to which the PC for Main drone is connected.
3. Run the VS project for the main drone.
