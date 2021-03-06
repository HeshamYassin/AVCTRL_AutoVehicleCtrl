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

%% Algorithm
% For 5s, Accelerate
% While imu1Orientation(3) > initialHeading - 90, Steering = 1
% For 4s, Steering = 0;
% While imu1Orientation > initialHeading - 180, Steering = 1
% For 5s, Steering = 0

%% Inputs
% stepOutputs - Outputs from Stepping Vehicle for Heading Orientation
% vehPosition - Vehicle Position for Stepping Vehicle
% startTime - Algorithm Starting Time
% stopTime - Algorithm Stop Time

function [handles] = UTurn(handles)
    
    for time = 0:handles.timeStep:5
        handles.Steering = 0;
        [stepOutput]=agx('step', [handles.moveVehicle, ...
                                  handles.vehiclePosition, ...
                                  handles.Throttle, handles.Gear, ...
                                  handles.Clutch, handles.Steering]);
        [handles] = sensorData(stepOutput, handles);
    end
    
    %% Get Initial Heading for Comparison
    initHeading = rad2deg(handles.imu1Orientation(3));
    
    %% Correct Heading to be within Limits [-180, 180]
    if (initHeading >= 180) 
        initHeading = initHeading - 360;
    elseif(initHeading <= -180)
        initHeading = initHeading + 360;
    end
    
    %% Get Current Heading for Turning Loop
    heading = rad2deg(handles.imu1Orientation(3));
    
    %% Correct Current Heading to be within Limits [-180, 180]
    if (heading >= 180) 
        heading = heading - 360;
    elseif(heading <= -180)
        heading = heading + 360;
    end
     
    %% Initialize Error and Integral for PID
    previousError = 0;
    previousIntegral = 0;
    
    error = deg2rad(-(initHeading - 90) + heading);
    
    while abs(error) > 1e-2
        %% PID Control
        error = deg2rad(-(initHeading - 90) + heading);
        
        [steeringAngle, error, integral] = PIDController(2, 0.01, 1, ...
        handles.timeStep, 'PID', previousError, error, previousIntegral);
        
        previousError = error;
        previousIntegral = integral;
        
        if steeringAngle <= -0.6109
            handles.Steering = -1;
        elseif steeringAngle >= 0.6109
            handles.Steering = 1;
        else
            handles.Steering = steeringAngle * 1.6369;
        end
        
        %% Algorithm
        [stepOutput]=agx('step', [handles.moveVehicle, ...
                                  handles.vehiclePosition, ...
                                  handles.Throttle, handles.Gear, ...
                                  handles.Clutch, handles.Steering]);
        [handles] = sensorData(stepOutput, handles);
       
        heading = rad2deg(handles.imu1Orientation(3));
        
        if (heading >= 180) 
            heading = heading - 360;
        elseif(heading <= -180)
            heading = heading + 360;
        end
        
        time = time + handles.timeStep;
    end
    
    for time = time:handles.timeStep:(time+1)
        handles.Steering = 0;
        [stepOutput]=agx('step', [handles.moveVehicle, ...
                                  handles.vehiclePosition, ...
                                  handles.Throttle, handles.Gear, ...
                                  handles.Clutch, handles.Steering]);
        [handles] = sensorData(stepOutput, handles);
    end

    %% Get Current Heading for Turning Loop
    heading = rad2deg(handles.imu1Orientation(3));
    
    %% Correct Current Heading to be within Limits [-180, 180]
    if (heading >= 180) 
        heading = heading - 360;
    elseif(heading <= -180)
        heading = heading + 360;
    end
    
    previousError = 0;
    previousIntegral = 0;
    
    error = deg2rad(-(initHeading - 180) + heading);
    
    while abs(error) > 1e-2
        %% PID Control
        error = deg2rad(-(initHeading - 180) + heading);
        
        [steeringAngle, error, integral] = PIDController(2, 0.01, 1, ...
        handles.timeStep, 'PID', previousError, error, previousIntegral);
        
        previousError = error;
        previousIntegral = integral;
        
        if steeringAngle <= -0.6109
            handles.Steering = -1;
        elseif steeringAngle >= 0.6109
            handles.Steering = 1;
        else
            handles.Steering = steeringAngle * 1.6369;
        end
        
        %% Algorithm
        [stepOutput]=agx('step', [handles.moveVehicle, ...
                                  handles.vehiclePosition, ...
                                  handles.Throttle, handles.Gear, ...
                                  handles.Clutch, handles.Steering]);
        [handles] = sensorData(stepOutput, handles);
        
        heading = rad2deg(handles.imu1Orientation(3));
        
        if (heading >= 180) 
            heading = heading - 360;
        elseif(heading <= -180)
            heading = heading + 360;
        end
        
        time = time + handles.timeStep;
    end
    
    for time = time:handles.timeStep:(time+5)
        handles.Steering = 0;
        [stepOutput]=agx('step', [handles.moveVehicle, ...
                                  handles.vehiclePosition, ...
                                  handles.Throttle, handles.Gear, ...
                                  handles.Clutch, handles.Steering]);
        [handles] = sensorData(stepOutput, handles);
    end
end