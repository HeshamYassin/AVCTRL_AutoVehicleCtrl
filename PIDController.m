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

function [u, error, integral] = PIDController(Kp, Ki, Kd, dt, type, prevError, error, prevIntegral)
    %% Error Stopping Criteria
    eps  = 1e-2;

    %% Integral Term
    if abs(error) > eps
        integral = prevIntegral + error * dt;
    else 
        integral = 0;
    end

    %% Derivative Term
    derivative = (error - prevError) / dt;

    %% Controller Type
    if strcmp(type , 'PID') 
        u = Kp * error + Ki * integral + Kd * derivative;
    elseif strcmp(type , 'PI') 
        u = Kp * error + Ki * integral;
    elseif strcmp(type , 'PD') 
        u = Kp * error + Kd * derivative;
    elseif strcmp(type , 'P') 
        u = Kp * error;
    end

end