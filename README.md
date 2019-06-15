# HTC Vive Leap Interaction Engine Integration
This repository modifies original Leap Motion interaction engine to integrates with HTC Vive hand tracking SDK.

In this initial stage, only example 0 is working correctly.

## Dependencies:
* HTC Vive hand track SDK 0.8.1
* Leap Motion core SDK

## How to use:
* Create an unity project and import HTC Vive hand track SDK 0.8.1 and Leap Motion core SDK.
* Change LeapMotion/Core/Scripts/HandModelManager.cs by this [HandModelManager.cs](https://gist.github.com/augustoicaro/0fe637b171eaadb21daf5a591f5a64fb)
* Clone this repository to LeapMotion/Modules/
* Open LeapMotion/Modules/HTC-Vive-Leap-Interaction-Engine-Integration/Examples/0. HTC Vive Simple

## Main configuration for a new project:
* Add a GestureProvider in main camera with mode = Skeleton
* Add an InteractionManager and adds two InteractionHands
* Set the Manager of each hand
* Add a HandModelManager
* Set the InteractionManager
* Add your preferred hand leap motion compatible
* Set each hand in HandModelManager
