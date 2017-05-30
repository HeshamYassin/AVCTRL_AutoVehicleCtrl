function [handles] = AVCTRL_vidInitAutoVehicleCtrl(handles)
%% Initialize Evironment Simulation Parameters
% Store Parameters into handles structure
% Initialize Vehicle Control Parameters
handles.Throttle = 0; % get from GUI
% handles.Throttle = str2double(get(handles.ThrottleParam, 'String'));
handles.Gear = 0; % Get from GUI
% handles.Gear = str2double(get(handles.GearParam, 'String'));
handles.Clutch = 0; % Get from GUI
% handles.Clutch = str2double(get(handles.ClutchParam, 'String'));
handles.Steering = 0; % Get from GUI
% handles.Steering = deg2rad(str2double(get(handles.SteeringParam, 'String'))) * (1/0.6109);    

% Initialize Vehicle's Position and Rotation
vehiclePositionX = 0; % Get from GUI
% vehiclePositionX = str2double(get(handles.VehPosX, 'String'));
vehiclePositionY = 0; % Get from GUI
% vehiclePositionY = str2double(get(handles.VehPosY, 'String'));
handles.vehiclePosition = [vehiclePositionX, vehiclePositionY];
handles.vehicleRotation = 0; % Get from GUI
% handles.vehicleRotation = str2double(get(handles.VehRotation, 'String'));
% Initialize Time
handles.time = 0;

% Set Simulation Time Step
% handles.timeStep = 0.01; % Get from GUI
handles.timeStep = str2double(get(handles.TimeStep, 'String'));

% Set Vehicle Motion
handles.moveVehicle = 0;

%% Show ECKE Logo
load Parameters
axes(handles.Logo);
imshow(Parameters)