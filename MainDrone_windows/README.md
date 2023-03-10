# Main drone
This is the Visual studio project for the main drone.  
We have only confirmed it works with DJI Mavic 2 pro.

## Prerequisites

* A device with Windows 10
* Visual Studio 2017
* Windows 10 SDK (Version 1803 or higher)
* Controller driver (Depends on your DJI products. Details are [here](https://developer.dji.com/windows-sdk/documentation/connection/Mavic2.html))


## Before running this project

### 1. Register as DJI Developer

Register for a DJI Developer account here.  
https://account.dji.com/register?appId=dji_sdk&backUrl=https%3A%2F%2Fdeveloper.dji.com%2Fuser&locale=en_US.

### 2. Generate an App Key

Issue an App key according to "Generate an App Key" section on the following page.  
https://developer.dji.com/windows-sdk/documentation/application-development-workflow/workflow-integrate.html  
In this project, I used "com.dji.wsdkdemo" for the package name. If you used another package name, please fill in the package name of Package.appxmanifest with your package name(Detailes are in Configutr Properties section of above page).

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