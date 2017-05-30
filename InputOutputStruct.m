%% Input
    %% Vehicle Intrinsics
    handles.Throttle %Input
    handles.Gear %Input
    handles.Clutch %Input
    handles.Steering %Input
    
    %% Pose Parameters
    handles.vehiclePosition %Input
    handles.vehicleRotation %Input
    
    %% Simulation Parameters
    handles.timeStep %Input
    handles.startTime %Input
    handles.stopTime %Input
    handles.moveVehicle
    
    %% Figure 8 Parameters
    handles.GoalX
    handles.GoalY
    handles.GoalHeading
    handles.numPoints
    
    %% Follow Line Parameters
    handles.LineA
    handles.LineB
    handles.LineC
    
    %% Follow Circle
    handles.CircleR
    
    %% GUI
    handles.output %GUI
    handles.ThrottleParam %GUI
    handles.GearParam %GUI
    hanldes.ClutchParam %GUI
    handles.SteeringParam %GUI
    handles.VehPosX %GUI
    handles.VehPosY %GUI
    handles.VehRotation %GUI
    handles.TimeStep %GUI
    handles.autopilot_gui %GUI
    handles.StartTime %GUI
    handles.StopTime %GUI
    handles.aLineParam
    handles.bLineParam
    handles.cLineParam
    handles.circleRParam
    handles.goalX
    handles.goalY
    handles.goalHeading
    
%% Output
    %% Sensor Data
    handles.numRays %Output
    handles.laserAngularRange %Output
    handles.laserDistanceRange %Output
    handles.engineRPM %Output
    handles.wheelAngularVelocity %Output
    handles.waistAngle %Output
    handles.laserDistance

        %% IMU-1
        handles.imu1Position %Output
        handles.imu1Orientation %Output
        handles.imu1Velocity %Output
        handles.imu1AngularVelocity %Output
        handles.imu1Acceleration %Output
        handles.imu1AngularAcceleration %Output

        %% IMU-2
        handles.imu2Position %Output
        handles.imu2Orientation %Output
        handles.imu2Velocity %Output
        handles.imu2AngularVelocity %Output
        handles.imu2Acceleration %Output
        handles.imu2AngularAcceleration %Output