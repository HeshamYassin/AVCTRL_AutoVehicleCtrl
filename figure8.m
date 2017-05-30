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

function [handles] = figure8(handles)
    for currPoint = 1:handles.numPoints
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
        Distance = abs( sqrt( (handles.GoalX(currPoint) - vehiclePositionX)^2 + (handles.GoalY(currPoint) - vehiclePositionY)^2 ) );

        %% Calculate Expected Heading
        expectedHeading = atan2( (handles.GoalY(currPoint) - vehiclePositionY) , (handles.GoalX(currPoint) - vehiclePositionX) );

        %% PID Controller Parameters
        previousError = 0;
        previousIntegral = 0;

    %% Control Loop untill Reaching Goal
        while Distance > 1     
            %% Alpha [-pi, pi]
            alpha = - expectedHeading + deg2rad(180);

            if (alpha >= pi) 
                alpha = alpha - 2*pi;
            elseif (alpha <= -pi)
                alpha = alpha + 2*pi;
            end

            %% Beta = - heading - ( - expected + 180 ) = expected - heading - 180
            beta = - deg2rad(Heading) - alpha;

            error = angleDiff( ( alpha + handles.GoalHeading(currPoint) ), beta );
            [steeringAngle, error, integral] = PIDController(2, 0.1, 1, ...
                                                             handles.timeStep, ...
                                                             'PID', ...
                                                             previousError, ...
                                                             error, previousIntegral);
            previousError = error;
            previousIntegral = integral;

            %% Correct Steering Angle
            if steeringAngle <= -0.6109
                handles.Steering = -1;
            elseif steeringAngle >= 0.6109
                handles.Steering = 1;
            else
                handles.Steering = steeringAngle * 1.6369;
            end

            %% Step Model
            [stepOutput] = agx('step', [handles.moveVehicle, ...
                                        handles.vehiclePosition, ...
                                        handles.Throttle, handles.Gear, ...
                                        handles.Clutch, handles.Steering]);
            [handles] = sensorData(stepOutput, handles);

            %% Sensor Feedback Data
            % Get Current Position
            vehiclePositionX = handles.imu1Position(1);
            vehiclePositionY = handles.imu1Position(2);
            handles.vehiclePosition = [vehiclePositionX, vehiclePositionY];

            % Get Current Heading
            Heading = rad2deg( handles.imu1Orientation(3) );

            % Correct Heading
            if (Heading >= 180) 
                Heading = Heading - 360;
            elseif(Heading <= -180)
                Heading = Heading + 360;
            end

            %% Convert to Polar
            % Calculate Distance
            Distance = abs( sqrt( ( handles.GoalX(currPoint) - vehiclePositionX )^2 + ( handles.GoalY(currPoint) - vehiclePositionY )^2 ) );

            % Calculate Expected Heading
            expectedHeading = atan2( (handles.GoalY(currPoint) - vehiclePositionY) , (handles.GoalX(currPoint) - vehiclePositionX) );

        end
    end    
end