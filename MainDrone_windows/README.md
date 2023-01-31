# Main drone
This is the Visual studio project for the main drone

## Prerequisites

* A device with Windows 10
* Visual Studio 2017
* Windows 10 SDK (Version 1803 or higher)
* Controller driver (depends on your DJI products. Details are [here](https://developer.dji.com/windows-sdk/documentation/connection/Mavic2.html))


## Before running this project

### 1. Register As DJI Developer

Register for a DJI Developer account here.  
https://account.dji.com/register?appId=dji_sdk&backUrl=https%3A%2F%2Fdeveloper.dji.com%2Fuser&locale=en_US.

### 2. Generate an App Key

Issue an App key according to "Generate an App Key" section on the following page.  
https://developer.dji.com/windows-sdk/documentation/application-development-workflow/workflow-integrate.html

### 3. Insert your App Key

Insert your App Key into DJISDKManager.Instance.RegisterApp("Please enter your App Key here.") in MainPage.xaml.cs.

### 4. Connect to wifi network

Connect your PC to the wifi network to which the PC for Follower drone is connected

### 5. Connect to controller

Connect your PC to the DJI remote controller