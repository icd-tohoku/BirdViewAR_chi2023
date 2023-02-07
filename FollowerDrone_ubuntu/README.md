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
Class to calculate the distance in GPS coordinates between two points using the Vincenty method.

### UndistortMtx.npz
File containing parameters for distortion correction.  
We calculated these parameters using cv2.calibrateCamera(), cv2.getOptimalNewCameraMatrix() and cv2.initUndistortRectifyMap().

### 3. Replace and import DJIVideoParser
Remove original DJIVideoParser and copy DJIVideoParser from [here](https://developer.dji.com/document/868bd800-1fc0-4f9e-a420-b7524a088909)  
Import DJIVideoParser project according to "Importing DJIVideoParser Project" section on the following page.  
https://developer.dji.com/document/868bd800-1fc0-4f9e-a420-b7524a088909

### 4. Insert your App Key

Insert your App Key into DJIWSDKFPVDeamo\DJISDKManager.Instance.RegisterApp("Please enter your App Key here.") in MainPage.xaml.cs.

### 5. Connect to wifi network

Connect your PC to the wifi network to which the PC for Follower drone is connected.

### 6. Connect to controller

Connect your PC to the DJI remote controller via Micro-USB cable.