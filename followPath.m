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
% Initial Position = Current Position
% Initial Heading = Current Heading
% Correct Initial Heading
% Calculate Distance
% Calculate Expected Heading
% for goalAngle = 0:360
%     GoalX = handles.circleRadius * cos( deg2rad(goalAngle) );
%     GoalY = handles.circleRadius * sin( deg2rad(goalAngle) );
%     while Distance > 0.5
%         %% PID Controller on Steering Angle
%         %% Correct Steering Angle
%         %% Step
%         %% Get Current Position
%         %% Calculate Distance
%         %% Calculate Expected Heading
%         %% Get Current Heading
%         %% Correct Heading
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial Position = Current Position
% Initial Heading = Current Heading
% Correct Initial Heading
% Calculate Distance
% Calculate Expected Heading
% frequency = 5/(2*pi*handles.circleRadius);
% for time = 0:handles.timeStep:60
%     GoalX = handles.circleRadius * cos( 2*pi*frequency*time );
%     GoalY = handles.circleRadius * sin( 2*pi*frequency*time );
%     while Distance > 0.5
%         %% PID Controller on Steering Angle
%         %% Correct Steering Angle
%         %% Step
%         %% Get Current Position
%         %% Calculate Distance
%         %% Calculate Expected Heading
%         %% Get Current Heading
%         %% Correct Heading
%     end
% end

function [handles] = followPath(handles)
    %% Initial Heading
    % initHeading = handles.imu1Orientation(3) + 90;
    
    %% Correct Initial Heading
    % if initHeading >= 180
    %      initHeading = initHeading - 360;
    % elseif initHeading <= -180
    %      initHeading = initHeading + 360;
    % end
    
    figure
    
    %% PID Controller Initial Parameters
    previousError = 0;
    previousIntegral = 0;

    %% Update Goal Position
    for time = 0:handles.timeStep:60 % theta = -pi/2:pi/1800:pi/2
        %% Initial Position = Current Position
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

        %% Calculate Frequency
        frequency = 5/(2*pi*handles.CircleR);

        %% Calculate Goal Position
        GoalX = handles.CircleR * cos( 2*pi*frequency*time - deg2rad(handles.vehicleRotation - 180) );
        GoalY = handles.CircleR * sin( 2*pi*frequency*time - deg2rad(handles.vehicleRotation - 180) );
        
        hold on
        plot(GoalX, GoalY, '-.R');
        axis([-100, 100, -100, 100]);
        
        %% Calculate Distance to Goal
        % Distance = abs( sqrt( (GoalX - vehiclePositionX)^2 + (GoalY - vehiclePositionY)^2 ) );

        %% System calculates the expected angle to always be positive, 
        % this is why when vehicle is in positive y, it rotates about the
        % same point. Since, it tries to follow the heading of expected
        % angle in +ve direction, it finds out that distance is increasing,
        % so it rotates to decrease, it finds out that heading is reversed,
        % and so on ...
        % If y difference is positive, reverse steering
        % else
        %% Calculate Expected Heading in Rad
        % if (GoalY - vehiclePositionY) < 0
        %     expectedHeading = atan2( (GoalY - vehiclePositionY) , (GoalX - vehiclePositionX) ) + deg2rad(180);
        % else
        %     expectedHeading = atan2( (GoalY - vehiclePositionY) , (GoalX - vehiclePositionX) ) - deg2rad(180);
        % end
        
        expectedHeading = atan2( (GoalY - vehiclePositionY) , (GoalX - vehiclePositionX) );
        %% End

        %% Control Loop untill Reaching Goal
%        while Distance > (0.15 * handles.CircleR)
            %% PID Controller on Steering Angle
            % error = deg2rad(Heading) - expectedHeading;
            error = angleDiff(expectedHeading, deg2rad(Heading));
            [steeringAngle, error, integral] = PIDController(2, 0.01, 1, ...
                                                             handles.timeStep, ...
                                                             'PID', ...
                                                             previousError, ...
                                                             error, previousIntegral);
            previousError = error;
            previousIntegral = integral;

%            steeringAngle = 5 * error;

            %% Correct Steering Angle
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

            hold on
            plot(vehiclePositionX, vehiclePositionY, '*g')
            %% Calculate Distance to Goal
            % Distance = abs( sqrt( (GoalX - vehiclePositionX)^2 + (GoalY - vehiclePositionY)^2 ) );

            %% System calculates the expected angle to always be positive, 
            % this is why when vehicle is in positive y, it rotates about the
            % same point. Since, it tries to follow the heading of expected
            % angle in +ve direction, it finds out that distance is increasing,
            % so it rotates to decrease, it finds out that heading is reversed,
            % and so on ...
            % If y difference is positive, reverse steering
            % else
            %% Calculate Expected Heading in Rad
            % if (GoalY - vehiclePositionY) < 0
            %     expectedHeading = atan2( (GoalY - vehiclePositionY) , (GoalX - vehiclePositionX) ) + deg2rad(180);
            % else
            %     expectedHeading = atan2( (GoalY - vehiclePositionY) , (GoalX - vehiclePositionX) ) - deg2rad(180);
            % end
            % expectedHeading = atan2( (GoalY - vehiclePositionY) , (GoalX - vehiclePositionX) );
            %% End

            %% Get Current Heading
            % Heading = rad2deg( handles.imu1Orientation(3) );

            %% Correct Heading
            % if (Heading >= 180) 
            %     Heading = Heading - 360;
            % elseif(Heading <= -180)
            %     Heading = Heading + 360;
            % end

%        end
    end
end