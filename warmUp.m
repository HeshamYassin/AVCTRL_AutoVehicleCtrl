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

function [stepOutput] = warmUp(handles)
    handles.Throttle = 0.3; % Open throttle 30% for warmUp
    handles.Gear = 1; % Neutral
    
    for x=0:handles.timeStep:1.01
        [stepOutput] = agx('step', [handles.moveVehicle, ...
                                    handles.vehiclePosition, ...
                                    handles.Throttle, handles.Gear, ...
                                    handles.Clutch, handles.Steering]);
    end
end