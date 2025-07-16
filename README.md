# UWE IPG Autonomous Simulator 2024-25

## Description
This package is the working project for UWE-AIs IPG CarMaker simulator; containing the tracks, scripts and ROS code in order to connect to the UWE-AI ADS.

## Current Features
1. Automated driver model: with a generated track, have the car automatically drive round for you
2. Cone detection: ground truth and noisy detections for blue, yellow, and orange coloured cones
3. ROS2 Humble integration: sim sensor data is available on ROS topics
1. Sensors:
    - GPS
    - Wheelspeeds
    - Steering angle

## Planned Features
1. Sensors:
    - IMU
    - SICK LiDARs / Ouster LiDARs
1. AI computer driving mode: sit back and watch your software take control
2. Manual driving mode: don't sit back and grab the wheel/speed-sliders, be sure to watch out for those kids trackside
1. RQT GUI for launching simulator; track_selection, driver model choice and more
2. State machine integration; experience software-to-software interaction with the car like never before
2. Weather modelling; rain or shine, we race
2. CAN communication; Can we do it? Maybe
3. UWEFS car modelling; or UWE-AI car modelling if we buy them out
4. City/countryside scenarios; experience the autonomous system in real-world environments (Business Plan/Real-World AI events)
