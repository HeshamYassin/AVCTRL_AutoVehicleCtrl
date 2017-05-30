%% About this Project
% Autonomous Vehicle AutoPilot System for Navigation in an Unknown Environment
% This project is copyright (c) for ECKE | THINK INNOVATION
% ECKE is an Egyptian Tech Company to provide Solutions to Accelerate
% Industrial Processes using Robotics and IoT
% ECKE AutoPilot System is RESTRICTED Intellictual Property, and no one is
% allowed to use, edit or produce this version for commercial use
% For more information and to contact us, please visit
% http://www.ecke-eg.com
% +20 (0) 102-011-5013

function [handles] = sensorData(stepOutput, handles)
    %% Read Engine RPM
    handles.engineRPM = stepOutput(:,1);
    
    %% Read Wheel Angular Velocity
    handles.wheelAngularVelocity = stepOutput(:,2:5);

    %% Read 1st IMU Data
    handles.imu1Position = stepOutput(:,6:8);
    handles.imu1Orientation = stepOutput(:,9:11);
    handles.imu1Velocity = stepOutput(:,12:14);
    handles.imu1AngularVelocity = stepOutput(:,15:17);
    handles.imu1Acceleration = stepOutput(:,18:20);
    handles.imu1AngularAcceleration = stepOutput(:,21:23);

    %% Read 2nd IMU Data
    handles.imu2Position = stepOutput(:,24:26);
    handles.imu2Orientation = stepOutput(:,27:29);
    handles.imu2Velocity = stepOutput(:,30:32);
    handles.imu2AngularVelocity = stepOutput(:,33:35);
    handles.imu2Acceleration = stepOutput(:,36:38);
    handles.imu2AngularAcceleration = stepOutput(:,39:41);

    %% Angle between front and rear body - Steering Angle [-35, 35]
    handles.waistAngle = stepOutput(:,42);

    %% Array of all distances for the laser
    handles.laserDistance = stepOutput(:,43:43+handles.numRays-1);
end