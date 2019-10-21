babots (battle robots)
======================
An idea which never graduated from the POC stage - merging both real world (using "robots") and mobile to create a different, more realistic gaming experience.
This project a combination of embedded (TI CC3220), mobile (Android) and server (Vert.x).

## Embedded
The embedded part used the CC3220SF LAUNCHXL launchpad board and a combination of two stepper motors controlled by a single TB6612FNG.
Most of the auxiliary code has been removed, leaving only the relevant code. The "babot client" is based on the MQTT client example, thus retains the original TI license.
API used across the code is FREE RTOS. 

## Server
The Server uses "websockets" (homebrewed in the embedded project) to communicate commands from the mobile devices to the embedded project.

## Android
The Android project simply uses a combo of "virtualjoystick" and "androidasync" to relay commands to the embedded project.