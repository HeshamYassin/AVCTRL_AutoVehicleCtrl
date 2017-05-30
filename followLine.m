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

%% Framework
% Initial Position = Current Position
% Initial Heading = Current Heading
% Correct Initial Heading
% Calculate Distance
% Calculate Expected Heading
% While Distance > 0.1
%   Control Over Current Heading and Expected Heading
%   Step
%   Get Current Position
%   Calculate Distance between Current Position and Goal Point
%   % Calculate Expected Heading
%   Get Current Heading
%   Correct Heading
% End

function [handles] = followLine(handles)
    %% Get Current Vehicle Position
    vehiclePositionX = handles.imu1Position(1);
    vehiclePositionY = handles.imu1Position(2);
    handles.vehiclePosition = [vehiclePositionX, vehiclePositionY];
    
    %% Get Current Vehicle Heading
    Heading = rad2deg( handles.imu1Orientation(3) );
    
    %% Correct Current Heading
    if (Heading >= 180)
        Heading = Heading - 360;
    elseif(Heading <= -180)
        Heading = Heading + 360;
    end
    
    %% Calculate Distance to Goal
    Distance = (handles.LineA * vehiclePositionX + ...
                     handles.LineB * vehiclePositionY + ...
                     handles.LineC) / sqrt(handles.LineA^2 + ...
                     handles.LineB^2);
    
    %% System calculates the expected angle to always be positive, 
    % this is why when vehicle is in positive y, it rotates about the
    % same point. Since, it tries to follow the heading of expected
    % angle in +ve direction, it finds out that distance is increasing,
    % so it rotates to decrease, it finds out that heading is reversed,
    % and so on ...
    % If y difference is positive, reverse steering
    % else
    %% Calculate Expected Heading in Rad
    expectedHeading = atan2( - handles.LineA , handles.LineB );% + deg2rad(180);
%     if (handles.GoalY - vehiclePositionY) < 0
%         expectedHeading = atan2( (handles.GoalY - vehiclePositionY) , (handles.GoalX - vehiclePositionX) ) + deg2rad(180);
%     else
%         expectedHeading = atan2( (handles.GoalY - vehiclePositionY) , (handles.GoalX - vehiclePositionX) ) - deg2rad(180);
%     end
    %% End

    %% Correct Expected Heading
    if (expectedHeading >= 180)
        expectedHeading = expectedHeading - 360;
    elseif(expectedHeading <= -180)
        expectedHeading = expectedHeading + 360;
    end
    
    %% PID Controller Initial Parameters
%     previousError = 0;
%     previousIntegral = 0;

    %% Showing Results
    lineEqX = -100:100;
    lineAng = tan( atan2( - handles.LineA, handles.LineB ) );
    lineEqY = lineAng * lineEqX + handles.LineC;
    figure;
    axis([-100, 100, -100, 100]);
    plot(lineEqX, lineEqY);
    
    %% Control Loop untill Reaching Goal
    while (1) %Distance > 0.1
        %% Controller on Steering Angle
        error = Heading - rad2deg(expectedHeading);
        
        if (error >= 180) 
            error = error - 360;
        elseif(error <= -180)
            error = error + 360;
        end

        % steeringAngle = - 0.05 * Distance + (mod( ( deg2rad(Heading) - expectedHeading ) + 3.1416 , 2*3.1416 ) - 3.1416 );
        % error = deg2rad(Heading) - expectedHeading;
%         [steeringAngle, error, integral] = PIDController(2, 0.01, 1, ...
%                                                          handles.timeStep, ...
%                                                          'PID', ...
%                                                          previousError, ...
%                                                          error, previousIntegral);
        steeringAngle = deg2rad( error - Distance );
                                                     
%         previousError = error;
%         previousIntegral = integral;
             
        %% Correct Heading Angle
        if steeringAngle <= -0.6109
            handles.Steering = -1;
        elseif steeringAngle >= 0.6109
            handles.Steering = 1;
        else
            handles.Steering = steeringAngle * 1.6369;
        end
       
        %% Step
        [stepOutput] = agx('step', [handles.moveVehicle, ...
                                    handles.vehiclePosition, ...
                                    handles.Throttle, handles.Gear, ...
                                    handles.Clutch, handles.Steering]);
        [handles] = sensorData(stepOutput, handles);
        
        %% Get Current Position
        vehiclePositionX = handles.imu1Position(1);
        vehiclePositionY = handles.imu1Position(2);
        handles.vehiclePosition = [vehiclePositionX, vehiclePositionY];
        
        %% Showing Results
        hold on
        plot(vehiclePositionX, vehiclePositionY, '*');
        
        %% Calculate Distance
        Distance = (handles.LineA * vehiclePositionX + ...
                     handles.LineB * vehiclePositionY + ...
                     handles.LineC) / sqrt(handles.LineA^2 + ...
                     handles.LineB^2);
        
        %% System calculates the expected angle to always be positive, 
        % this is why when vehicle is in positive y, it rotates about the
        % same point. Since, it tries to follow the heading of expected
        % angle in +ve direction, it finds out that distance is increasing,
        % so it rotates to decrease, it finds out that heading is reversed,
        % and so on ...
        % If y difference is positive, reverse steering
        % else
        %% Calculate Expected Heading
%         expectedHeading = atan2( -handles.LineA , handles.LineB );% - deg2rad(180);
%         if (handles.GoalY - vehiclePositionY) < 0
%             expectedHeading = atan2( (handles.GoalY - vehiclePositionY) , (handles.GoalX - vehiclePositionX) ) + deg2rad(180);
%         else
%             expectedHeading = atan2( (handles.GoalY - vehiclePositionY) , (handles.GoalX - vehiclePositionX) ) - deg2rad(180);
%         end
        %% End
        
        %% Get Current Heading
        Heading = rad2deg( handles.imu1Orientation(3) );
        
        %% Correct Heading
        if (Heading >= 180) 
            Heading = Heading - 360;
        elseif(Heading <= -180)
            Heading = Heading + 360;
        end
        
    end
    
%     %% Keep Following the Line
%     while (1) % Close is pressed
%     [stepOutput] = agx('step', [handles.moveVehicle, ...
%                                 handles.vehiclePosition, ...
%                                 handles.Throttle, handles.Gear, ...
%                                 handles.Clutch, handles.Steering]);
%     [handles] = sensorData(stepOutput, handles);
%     end
end