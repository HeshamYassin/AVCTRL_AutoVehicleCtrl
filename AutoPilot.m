function varargout = AutoPilot(varargin)
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

%% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @AutoPilot_OpeningFcn, ...
                   'gui_OutputFcn',  @AutoPilot_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
%% End initialization code - DO NOT EDIT


%% --- Executes just before AutoPilot is made visible.
function AutoPilot_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to AutoPilot (see VARARGIN)

% Choose default command line output for AutoPilot
handles.output = hObject;

%% Initialize Evironment Simulation Parameters
handles = AVCTRL_vidInitAutoVehicleCtrl(handles);

%% Update handles structure
guidata(hObject, handles);

% UIWAIT makes AutoPilot wait for user response (see UIRESUME)
% uiwait(handles.autopilot_gui);


% --- Outputs from this function are returned to the command line.
function varargout = AutoPilot_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in load.
function load_Callback(hObject, eventdata, handles)
% hObject    handle to load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Reset Environment
agx('reset');

% Start Simulation Server
agx('visual', 1);

%% Initialize the scene with intial values and the path to the .agxLua script
% Initialize Vehicle Control Parameters
% handles.Throttle = 0; % get from GUI
handles.Throttle = str2double(get(handles.ThrottleParam, 'String'));
% handles.Gear = 0; % Get from GUI
handles.Gear = str2double(get(handles.GearParam, 'String'));
% handles.Clutch = 0; % Get from GUI
handles.Clutch = str2double(get(handles.ClutchParam, 'String'));
% handles.Steering = 0; % Get from GUI
handles.Steering = deg2rad(str2double(get(handles.SteeringParam, 'String'))) * (1/0.6109);

% Initialize Vehicle's Position and Rotation
% vehiclePositionX = 0; % Get from GUI
vehiclePositionX = str2double(get(handles.VehPosX, 'String'));
% vehiclePositionY = 0; % Get from GUI
vehiclePositionY = str2double(get(handles.VehPosY, 'String'));
handles.vehiclePosition = [vehiclePositionX, vehiclePositionY];
% handles.vehicleRotation = 0; % Get from GUI
handles.vehicleRotation = str2double(get(handles.VehRotation, 'String'));

% Set Simulation Time Step
% handles.timeStep = 0.01; % Get from GUI
handles.timeStep = str2double(get(handles.TimeStep, 'String'));

% Load Simulation Environment with Initial Values
% initOutput - Contain output data from the init phase
% sizeStepInput - Number of input elements required for the 'step' phase.
% sizeStepOutput - Number of output elements available for the 'step' phase
% [sizeInitInput, initOutput, sizeStepInput, sizeStepOutput] = agx('load', 'F:/Algoryx/AgX-2.15.0.3/matlab-terrainVehicle.agxLua',[handles.vehiclePosition handles.vehicleRotation], handles.timeStep);
[sizeInitInput, initOutput, sizeStepInput, sizeStepOutput] = agx('load', '.../terrainVehicle.agxLua',[handles.vehiclePosition handles.vehicleRotation], handles.timeStep);

%% Store LRF Data in handles Structure
handles.numRays = initOutput(:,1); % Num rays in laser system
handles.laserAngularRange = initOutput(:,2); % Range (in radians) for the laser range finder
handles.laserDistanceRange = initOutput(:,3); % Range in meters for the laser range finder

%% Warm Up the Vehicle by running Simulation Environment for 1s
% If [stepOutput] not needed, keep it local in the warmUp function
[stepOutput] = warmUp(handles);

%% Update handles Structure
[handles] = sensorData(stepOutput, handles);

%% Update handles Structure
guidata(hObject, handles);

% --- Executes on button press in close.
function close_Callback(hObject, eventdata, handles)
% hObject    handle to close (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
agx('reset');
close(handles.autopilot_gui);


% --- Executes on key press with focus on autopilot_gui and none of its controls.
function autopilot_gui_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to autopilot_gui (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)
nameC=DetectKeyboard(hObject,eventdata,handles);

switch nameC
    case 'w'
        handles.Steering = 0;
        handles.Gear = 2;
        % Keep Limited by the User Parameters
        handles.Throttle = str2double(get(handles.ThrottleParam, 'String'));
        % Keep Limited by the User Parameters
        handles.Clutch = str2double(get(handles.ClutchParam, 'String'));
    case 's'
        handles.Steering = handles.Steering;
        handles.Gear = 1;
        handles.Throttle = 0;
        handles.Clutch = 0;
    case 'd'
        handles.Steering = 1;
        handles.Gear = handles.Gear;
        handles.Throttle = handles.Throttle;
        handles.Clutch = handles.Clutch;
    case 'a'
        handles.Steering = -1;
        handles.Gear = handles.Gear;
        handles.Throttle = handles.Throttle;
        handles.Clutch = handles.Clutch;
    case 'x'
        handles.Steering = 0;
        handles.Gear = 0;
        handles.Throttle = 0.5;
        handles.Clutch = 1;
    case 'e'
        handles.Steering = 1;
        handles.Gear = 2;
        % Keep Limited by the User Parameters
        handles.Throttle = str2double(get(handles.ThrottleParam, 'String'));
        % Keep Limited by the User Parameters
        handles.Clutch = str2double(get(handles.ClutchParam, 'String'));
    case 'q'
        handles.Steering = -1;
        handles.Gear = 2;
        % Keep Limited by the User Parameters
        handles.Throttle = str2double(get(handles.ThrottleParam, 'String'));
        % Keep Limited by the User Parameters
        handles.Clutch = str2double(get(handles.ClutchParam, 'String'));
    case 'z'
        handles.Steering = -1;
        handles.Gear = 0;
        handles.Throttle = 0.5;
        handles.Clutch = 1;
    case 'c'
        handles.Steering = 1;
        handles.Gear = 0;
        handles.Throttle = 0.5;
        handles.Clutch = 1;
end

%% Update Position and Orientation
% vehiclePositionX = handles.vehiclePosition(1) + handles.imu1Position(1);
vehiclePositionX = handles.imu1Position(1);
% vehiclePositionY = handles.vehiclePosition(2) + handles.imu1Position(2);
vehiclePositionY = handles.imu1Position(2);
handles.vehiclePosition = [vehiclePositionX vehiclePositionY];
% handles.vehicleRotation = handles.vehicleRotation + handles.imu1Orientation(3);
% handles.vehicleRotation = handles.imu1Orientation(3);

%% Step AutoPilot Controller
[stepOutput]=agx('step', [handles.moveVehicle, handles.vehiclePosition, handles.Throttle, handles.Gear, handles.Clutch, handles.Steering]);

%% Read Sensor Data
[handles] = sensorData(stepOutput, handles);

%% Update handles Structure
guidata(hObject,handles);

%% Vehicle Position X GUI Text
function VehPosX_Callback(hObject, eventdata, handles)
% hObject    handle to VehPosX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Get the new pvalue for the Gain.

% Hints: get(hObject,'String') returns contents of VehPosX as text
%        str2double(get(hObject,'String')) returns contents of VehPosX as a double

posx = str2double(get(hObject,'String'));
VehPosXMin = get(hObject, 'Min');
VehPosXMax = get(hObject, 'Max');
if  isnan(posx) || (posx < VehPosXMin) || (posx > VehPosXMax),
    % Revert to last pvalue, as indicated by KfValueSlider.
    if posx < VehPosXMin
        % posx = VehPosXMin;
        set(hObject, 'String', VehPosXMin)
    elseif posx > VehPosXMax
        % posx = VehPosXMax;
        set(hObject, 'String', VehPosXMax)
    else
        % posx = 0; % Default
        set(hObject, 'String', '0')
    end
%else
%    handles.vehiclePositionX = posx;
end

%% Update handles Structure
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function VehPosX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to VehPosX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%% Vehicle Position Y GUI Text
function VehPosY_Callback(hObject, eventdata, handles)
% hObject    handle to VehPosY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of VehPosY as text
%        str2double(get(hObject,'String')) returns contents of VehPosY as a double

posy = str2double(get(hObject,'String'));
VehPosYMin = get(hObject, 'Min');
VehPosYMax = get(hObject, 'Max');
if  isnan(posy) || (posy < VehPosYMin) || (posy > VehPosYMax),
    % Revert to last pvalue, as indicated by KfValueSlider.
    if posy < VehPosYMin
        % posy = VehPosYMin;
        set(hObject, 'String', VehPosYMin)
    elseif posy > VehPosYMax
        % posy = VehPosYMax;
        set(hObject, 'String', VehPosYMax)
    else
        % posy = 0; % Default
        set(hObject, 'String','0')
    end
%else
%    handles.vehiclePositionY = posy;
end

% Update handles Structure
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function VehPosY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to VehPosY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%% Vehicle Rotation GUI Text
function VehRotation_Callback(hObject, eventdata, handles)
% hObject    handle to VehRotation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of VehRotation as text
%        str2double(get(hObject,'String')) returns contents of VehRotation as a double

rot = str2double(get(hObject,'String'));
VehRotationMin = get(hObject, 'Min');
VehRotationMax = get(hObject, 'Max');
if  isnan(rot) || (rot < VehRotationMin) || (rot > VehRotationMax),
    % Revert to last pvalue, as indicated by KfValueSlider.
    if rot < VehRotationMin
        % rot = VehRotationMin;
        set(hObject, 'String', VehRotationMin)
    elseif rot > VehRotationMax
        % rot = VehRotationMax;
        set(hObject, 'String', VehRotationMax)
    else
        % rot = 0; % Default
        set(hObject, 'String', '0')
    end
%else
%    handles.vehicleRotation = rot;
end

% Update handles Structure
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function VehRotation_CreateFcn(hObject, eventdata, handles)
% hObject    handle to VehRotation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function TimeStep_Callback(hObject, eventdata, handles)
% hObject    handle to TimeStep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of TimeStep as text
%        str2double(get(hObject,'String')) returns contents of TimeStep as a double

timeStep = str2double(get(hObject, 'String'));
timeStepMin = get(hObject, 'Min');
if  isnan(timeStep) || (timeStep <= timeStepMin)
    set(hObject, 'String', timeStepMin+0.01)
%else
%    handles.timeStep = timeStep;
end

%% Update handles Structure
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function TimeStep_CreateFcn(hObject, eventdata, handles)
% hObject    handle to TimeStep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ClutchParam_Callback(hObject, eventdata, handles)
% hObject    handle to ClutchParam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ClutchParam as text
%        str2double(get(hObject,'String')) returns contents of ClutchParam as a double

Clutch = str2double(get(hObject,'String'));
ClutchMin = get(hObject, 'Min');
ClutchMax = get(hObject, 'Max');

if  isnan(Clutch) || (Clutch < ClutchMin) || (Clutch > ClutchMax),
    % Revert to last pvalue, as indicated by KfValueSlider.
    if Clutch < ClutchMin
        % Clutch = ClutchMin;
        set(hObject, 'String', ClutchMin)
    elseif Clutch > ClutchMax
        % Clutch = ClutchMax;
        set(hObject, 'String', ClutchMax)
    else
        % Clutch = 0;
        set(hObject, 'String', '0')
    end
% else
%     handles.Clutch = Clutch;    
end

%% Update handles Structure
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function ClutchParam_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ClutchParam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function SteeringParam_Callback(hObject, eventdata, handles)
% hObject    handle to SteeringParam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of SteeringParam as text
%        str2double(get(hObject,'String')) returns contents of SteeringParam as a double

Steering = str2double(get(hObject,'String'));
SteeringMin = get(hObject, 'Min');
SteeringMax = get(hObject, 'Max');

if  isnan(Steering) || (Steering < SteeringMin) || (Steering > SteeringMax),
    % Revert to last pvalue, as indicated by KfValueSlider.
    if Steering < SteeringMin % -35
        % Steering = -35;
        set(hObject, 'String', SteeringMin)
    elseif Steering > SteeringMax % 35
        % Steering = 35;
        set(hObject, 'String', SteeringMax)
    else
        % Steering = 0;
        set(hObject, 'String', '0')
    end
% else
%     Steering = deg2rad(Steering) * (1/0.6109);
%     handles.Steering = Steering;    
end

%% Update handles Structure
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function SteeringParam_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SteeringParam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function GearParam_Callback(hObject, eventdata, handles)
% hObject    handle to GearParam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of GearParam as text
%        str2double(get(hObject,'String')) returns contents of GearParam as a double

Gear = str2double(get(hObject,'String'));
GearMin = get(hObject, 'Min');
GearMax = get(hObject, 'Max');

if  isnan(Gear) || (Gear < GearMin) || (Gear > GearMax),
    % Revert to last pvalue, as indicated by KfValueSlider.
    if Gear < GearMin
        % Gear = GearMin;
        set(hObject, 'String', GearMin)
    elseif Gear > GearMax
        % Gear = GearMax;
        set(hObject, 'String', GearMax)
    else
        % Gear = 0;
        set(hObject, 'String', '0')
    end
% else
%     handles.Gear = Gear;    
end

%% Update handles Structure
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function GearParam_CreateFcn(hObject, eventdata, handles)
% hObject    handle to GearParam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ThrottleParam_Callback(hObject, eventdata, handles)
% hObject    handle to ThrottleParam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ThrottleParam as text
%        str2double(get(hObject,'String')) returns contents of ThrottleParam as a double

Throttle = str2double(get(hObject,'String'));
ThrottleMin = get(hObject, 'Min');
ThrottleMax = get(hObject, 'Max');

if  isnan(Throttle) || (Throttle < ThrottleMin) || (Throttle > ThrottleMax),
    % Revert to last pvalue, as indicated by KfValueSlider.
    if Throttle < ThrottleMin
        % Throttle = ThrottleMin;
        set(hObject, 'String', ThrottleMin)
    elseif Throttle > ThrottleMax
        % Throttle = ThrottleMax;
        set(hObject, 'String', ThrottleMax)
    else
        % Throttle = 0;
        set(hObject, 'String','0')
    end
% else
%     handles.Throttle = Throttle;    
end

%% Update handles Structure
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function ThrottleParam_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ThrottleParam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function StartTime_Callback(hObject, eventdata, handles)
% hObject    handle to StartTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of StartTime as text
%        str2double(get(hObject,'String')) returns contents of StartTime as a double

startTimeVal = str2double(get(hObject,'String'));
if isnan(startTimeVal) || (startTimeVal <= 0)
    set(hObject, 'String', '0')
% else
%     handles.startTime = startTimeVal;
end

%% Updating handles Structure
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function StartTime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to StartTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function StopTime_Callback(hObject, eventdata, handles)
% hObject    handle to StopTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of StopTime as text
%        str2double(get(hObject,'String')) returns contents of StopTime as a double

stopTimeVal = str2double(get(hObject,'String'));
if isnan(stopTimeVal) || (stopTimeVal <= 0)
    set(hObject, 'String', '0')
% else
%     handles.stopTime = stopTimeVal;
end

%% Updating handles Structure
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function StopTime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to StopTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in changeLanesCMD.
function changeLanesCMD_Callback(hObject, eventdata, handles)
% hObject    handle to changeLanesCMD (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% Get Start Time
handles.startTime = str2double(get(handles.StartTime, 'String'));

%% Get Stop Time
handles.stopTime = str2double(get(handles.StopTime, 'String'));

%% Initialize Input Vector for Data Logging


%% Update Position and Orientation
% vehiclePositionX = handles.vehiclePosition(1) + handles.imu1Position(1);
vehiclePositionX = handles.imu1Position(1);
% vehiclePositionY = handles.vehiclePosition(2) + handles.imu1Position(2);
vehiclePositionY = handles.imu1Position(2);
handles.vehiclePosition = [vehiclePositionX vehiclePositionY];
% handles.vehicleRotation = handles.vehicleRotation + handles.imu1Orientation(3);
% handles.vehicleRotation = handles.imu1Orientation(3);

%% Change Lanes
[handles] = changeLanes(handles);

%% Updating handles Structure
guidata(hObject,handles)

% --- Executes on button press in uTurnCMD.
function uTurnCMD_Callback(hObject, eventdata, handles)
% hObject    handle to uTurnCMD (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% Get Start Time
handles.startTime = str2double(get(handles.StartTime, 'String'));

%% Get Stop Time
handles.stopTime = str2double(get(handles.StopTime, 'String'));

%% Initialize Input Vector for Data Logging


%% Update Position and Orientation
% vehiclePositionX = handles.vehiclePosition(1) + handles.imu1Position(1);
vehiclePositionX = handles.imu1Position(1);
% vehiclePositionY = handles.vehiclePosition(2) + handles.imu1Position(2);
vehiclePositionY = handles.imu1Position(2);
handles.vehiclePosition = [vehiclePositionX vehiclePositionY];
% handles.vehicleRotation = handles.vehicleRotation + handles.imu1Orientation(3);
% handles.vehicleRotation = handles.imu1Orientation(3);

%% Change Lanes
[handles] = UTurn(handles);

%% Updating handles Structure
guidata(hObject,handles)

% --- Executes on button press in figure8CMD.
function figure8CMD_Callback(hObject, eventdata, handles)
% hObject    handle to figure8CMD (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% Load Path Points
load Path

%% Assign Path Points to [x, y, heading]
handles.GoalX = Path(:, 1);
handles.GoalY = Path(:, 2);
handles.GoalHeading = deg2rad( Path(:, 3) );

%% Number of Points
handles.numPoints = length(Path);

%% Figure 8
[handles] = figure8(handles);

%% Updating handles Structure
guidata(hObject,handles)

function aLineParam_Callback(hObject, eventdata, handles)
% hObject    handle to aLineParam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of aLineParam as text
%        str2double(get(hObject,'String')) returns contents of aLineParam as a double


% --- Executes during object creation, after setting all properties.
function aLineParam_CreateFcn(hObject, eventdata, handles)
% hObject    handle to aLineParam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function bLineParam_Callback(hObject, eventdata, handles)
% hObject    handle to bLineParam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of bLineParam as text
%        str2double(get(hObject,'String')) returns contents of bLineParam as a double


% --- Executes during object creation, after setting all properties.
function bLineParam_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bLineParam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function cLineParam_Callback(hObject, eventdata, handles)
% hObject    handle to cLineParam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cLineParam as text
%        str2double(get(hObject,'String')) returns contents of cLineParam as a double


% --- Executes during object creation, after setting all properties.
function cLineParam_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cLineParam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function circleRParam_Callback(hObject, eventdata, handles)
% hObject    handle to circleRParam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of circleRParam as text
%        str2double(get(hObject,'String')) returns contents of circleRParam as a double

circler = str2double(get(hObject,'String'));
CircleRMin = get(hObject, 'Min');
CircleRMax = get(hObject, 'Max');
if  isnan(circler) || (circler < CircleRMin) || (circler > CircleRMax),
    % Revert to last pvalue, as indicated by KfValueSlider.
    if circler < CircleRMin
        % circler = CircleRMin;
        set(hObject, 'String', CircleRMin)
    elseif circler > CircleRMax
        % circler = CircleRMax;
        set(hObject, 'String', CircleRMax)
    else
        % circler = 0; % Default
        set(hObject, 'String', '0')
    end
%else
%    handles.CircleR = circler;
end

%% Update handles Structure
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function circleRParam_CreateFcn(hObject, eventdata, handles)
% hObject    handle to circleRParam (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in followLineCMD.
function followLineCMD_Callback(hObject, eventdata, handles)
% hObject    handle to followLineCMD (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% Get Line Parameters
% Get Line a
handles.LineA = str2double(get(handles.aLineParam, 'String'));
% Get Line b
handles.LineB = str2double(get(handles.bLineParam, 'String'));
% Get Line c
handles.LineC = str2double(get(handles.cLineParam, 'String'));

%% Call Function
[handles] = followLine(handles);

%% Update handles Structure
guidata(hObject,handles);

% --- Executes on button press in followCircleCMD.
function followCircleCMD_Callback(hObject, eventdata, handles)
% hObject    handle to followCircleCMD (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% Get Goal Radius
% Get Goal R
handles.CircleR = str2double(get(handles.circleRParam, 'String'));

%% Call Function
[handles] = followPath(handles);

%% Update handles Structure
guidata(hObject,handles);

function goalX_Callback(hObject, eventdata, handles)
% hObject    handle to goalX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of goalX as text
%        str2double(get(hObject,'String')) returns contents of goalX as a double

goalx = str2double(get(hObject,'String'));
GoalXMin = get(hObject, 'Min');
GoalXMax = get(hObject, 'Max');
if  isnan(goalx) || (goalx < GoalXMin) || (goalx > GoalXMax),
    % Revert to last pvalue, as indicated by KfValueSlider.
    if goalx < GoalXMin
        % goalx = GoalXMin;
        set(hObject, 'String', GoalXMin)
    elseif goalx > GoalXMax
        % goalx = GoalXMax;
        set(hObject, 'String', GoalXMax)
    else
        % goalx = 0; % Default
        set(hObject, 'String', '0')
    end
%else
%    handles.GoalX = goalx;
end

%% Update handles Structure
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function goalX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to goalX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function goalY_Callback(hObject, eventdata, handles)
% hObject    handle to goalY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of goalY as text
%        str2double(get(hObject,'String')) returns contents of goalY as a double

goaly = str2double(get(hObject,'String'));
GoalYMin = get(hObject, 'Min');
GoalYMax = get(hObject, 'Max');
if  isnan(goaly) || (goaly < GoalYMin) || (goaly > GoalYMax),
    % Revert to last pvalue, as indicated by KfValueSlider.
    if goaly < GoalYMin
        % goaly = GoalYMin;
        set(hObject, 'String', GoalYMin)
    elseif goaly > GoalYMax
        % goaly = GoalYMax;
        set(hObject, 'String', GoalYMax)
    else
        % goaly = 0; % Default
        set(hObject, 'String', '0')
    end
%else
%    handles.GoalY = goaly;
end

%% Update handles Structure
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function goalY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to goalY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function goalHeading_Callback(hObject, eventdata, handles)
% hObject    handle to goalHeading (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of goalHeading as text
%        str2double(get(hObject,'String')) returns contents of goalHeading as a double


% --- Executes during object creation, after setting all properties.
function goalHeading_CreateFcn(hObject, eventdata, handles)
% hObject    handle to goalHeading (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in movePointCMD.
function movePointCMD_Callback(hObject, eventdata, handles)
% hObject    handle to movePointCMD (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% Get Goal Position
% Get Goal X
handles.GoalX = str2double(get(handles.goalX, 'String'));
% Get Goal Y
handles.GoalY = str2double(get(handles.goalY, 'String'));

%% Call Function
[handles] = movePoint(handles);

%% Update handles Structure
guidata(hObject,handles);

% --- Executes on button press in movePoseCMD.
function movePoseCMD_Callback(hObject, eventdata, handles)
% hObject    handle to movePoseCMD (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% Get Goal Position
% Get Goal X
handles.GoalX = str2double(get(handles.goalX, 'String'));
% Get Goal Y
handles.GoalY = str2double(get(handles.goalY, 'String'));
% Get Goal Heading
handles.GoalHeading = str2double(get(handles.goalHeading, 'String'));

%% Call Function
[handles] = movePose(handles);

%% Update handles Structure
guidata(hObject,handles);


% --- Executes on button press in moreInfo.
function moreInfo_Callback(hObject, eventdata, handles)
% hObject    handle to moreInfo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
run('About.m');
