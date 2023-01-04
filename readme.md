# Team 3161's offseason swerve project

The project involves building a swerve robot for the First Robotics Competition (FRC) season. This is the team's first time working with swerve technology and we have used code from team 3512 as a starting point. We have made updates to this code base in order to customize the robot for their needs. The goal of the project is to have a fully functional swerve robot ready before the new(2023) FRC season gets announced.

## Goals

 - ✅ A fully functional swerve drive
 - ✅ Planning Autos using PathPlanner
 - ✅ Rotation correction while operating the robot
 - ✅ April Tag detection using LimeLight V2
 - ✅ April Tag centering (The robot rotates towards them)
 - ✅ Live PID tuning
 - ✅ Use DPads on the controller to point the robot towards the values relative to the field

## Hardware
- **Swerve**
  - MK4i L2 modules
  - CANCoder
  - Canivore (optional)
  - Pigeon2
- **Vision**
	- LimeLight V2 using gloworm and photonvision

## Demo
### Auto
### Vision
### Teleop

## Setup
Setup would be straight forward if your team is using the same hardware as long as you put in the correct values based on your robot in the Constants.java

 1. Clone the repository
 2. Don't forget to change the team number!!
 3. Here are the list of values that you'll need to change before enabling the robot
 
|Value|Location|Description|
|--|--|--|
|cameraName|Constants->Vision|The name of your camera, can be found in photonvision's dashboard, [documentation](https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html#what-is-a-photoncamera)|
|pigeonID|Constants->Swerve|Pigeon2 ID|
|pigeonCanBUS|Constants->Swerve|Put in "rio" if your pigeon is on the main loop. If it's on a canivore, put in whatever canbus your canivore is set to|
|trackWidth|Constants->Swerve|the length of your robot|
|wheelBase|Constants->Swerve|the width of your robot|
|wheelDiameter|Constants->Swerve|the wheel diameter, can be found in your swerve module's product page|
|driverGearRatio|Constants->Swerve|the gear reduction for drive motors, can be found in your swerve module's product page|
|angleGearRatio|Constants->Swerve|the gear reduction for angle motors, can be found in your swerve module's product page|
|Mod0, Mod1, Mod2, Mod3|Constants->Swerve|Swerve module constats, starting from front left, front right, back left, back right in order.|
|driveMotorID|Constants->Swerve->Mod|Drive motor id for each module|
|angleMotorID|Constants->Swerve->Mod|Angle motor id for each module|
|driveMotorID|Constants->Swerve->Mod|Drive motor id for each module|
|canCoderID|Constants->Swerve->Mod|Can coder id for each module|
|cancoderCANBUS|Constants->Swerve->Mod|Drive motor id for each module|
|driveMotorID|Constants->Swerve->Mod|Drive motor id for each module|
|driveMotorID|Constants->Swerve->Mod|Drive motor id for each module|
|driveMotorID|Constants->Swerve->Mod|Put in “rio” if your pigeon is on the main loop. If it’s on a canivore, put in whatever canbus your canivore is set to|
|angleOffset|Constants->Swerve->Mod|After setting up the code, print out the cancoder values of each module, then straighten the wheels towards the front of the robot in a way the modules pinions are point outwards. Then put in the offset value for each module|

## Programmers
Pooria Ahmadi ☕❤️👨‍💻
 - [Website](https://pooria.tech)
 - [Github](https://github.com/pooriaahmadi)
