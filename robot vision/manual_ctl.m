function varargout = manual_ctl(varargin)
% MANUAL_CTL MATLAB code for manual_ctl.fig
%      MANUAL_CTL, by itself, creates a new MANUAL_CTL or raises the existing
%      singleton*.
%
%      H = MANUAL_CTL returns the handle to a new MANUAL_CTL or the handle to
%      the existing singleton*.
%
%      MANUAL_CTL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MANUAL_CTL.M with the given input arguments.
%
%      MANUAL_CTL('Property','Value',...) creates a new MANUAL_CTL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before manual_ctl_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to manual_ctl_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help manual_ctl

% Last Modified by GUIDE v2.5 09-Sep-2016 18:44:28

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @manual_ctl_OpeningFcn, ...
                   'gui_OutputFcn',  @manual_ctl_OutputFcn, ...
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
% End initialization code - DO NOT EDIT


% --- Executes just before manual_ctl is made visible.
function manual_ctl_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to manual_ctl (see VARARGIN)

% Choose default command line output for manual_ctl
handles.output = hObject;

if nargin == 4
    handles.v = varargin{:};
    handles.hview = imshow(handles.v.getsnapshot(), 'Parent', handles.axes1);
    
    home = [25e-2 0 10e-2 pi/2-0.01];
    g = get(handles.pick_place, 'Value') * 100;

    set(handles.xctrl, 'Value', interp1([5e-2 30e-2], [0 1], home(1), 'linear'))
    set(handles.yctrl, 'Value', interp1([-15e-2 15e-2], [0 1], home(2), 'linear'))
    set(handles.zctrl, 'Value', interp1([0 35e-2], [0 1], home(3), 'linear'))
    set(handles.thctrl, 'Value', interp1([-pi/2 pi/2], [0 1], home(4), 'linear'))
    
    handles.v.move(home, g)
end

handles.timer = timer(...
    'ExecutionMode', 'fixedRate', ...   % Run timer repeatedly
    'Period', 1/10, ...                % Initial period is 1 sec.
    'TimerFcn', {@update_display, hObject, handles}); % Specify callback
start(handles.timer)



% Update handles structure
guidata(hObject, handles);

% UIWAIT makes manual_ctl wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = manual_ctl_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function update_display(obj, event, hObject, handles)
set(handles.hview, 'CData', handles.v.getsnapshot())

% --- Executes on slider movement.
function xctrl_Callback(hObject, eventdata, handles)
% hObject    handle to xctrl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
move_robot(handles)

function [pose, g] = get_position(handles)
x = interp1([0 1], [10e-2 40e-2], get(handles.xctrl, 'Value'), 'linear');
y = interp1([0 1], [-15e-2 15e-2], get(handles.yctrl, 'Value'), 'linear');
z = interp1([0 1], [0 35e-2], get(handles.zctrl, 'Value'), 'linear');
th = interp1([0 1], [-pi/2 pi/2], get(handles.thctrl, 'Value'), 'linear');
g = get(handles.pick_place, 'Value') * 100;
pose = [x y z th];

function move_robot(handles)
[pose, g] = get_position(handles);
handles.v.move(pose, g)

% --- Executes during object creation, after setting all properties.
function xctrl_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xctrl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end




% --- Executes on button press in pick_place.
function pick_place_Callback(hObject, eventdata, handles)
% hObject    handle to pick_place (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[pos, gstate] = get_position(handles);
z0 = pos(3);
gstate0 = (~gstate) * 100;

pos(3) = 0.02;
handles.v.move(pos, gstate0);
pause(0.5)

handles.v.move(pos, gstate);
pause(1)

move_robot(handles)

% --- Executes on button press in return_home.
function return_home_Callback(hObject, eventdata, handles)
% hObject    handle to return_home (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
home = [25e-2 0 10e-2 pi/2-0.01];
g = get(handles.pick_place, 'Value') * 100;

set(handles.xctrl, 'Value', interp1([10e-2 40e-2], [0 1], home(1), 'linear'))
set(handles.yctrl, 'Value', interp1([-15e-2 15e-2], [0 1], home(2), 'linear'))
set(handles.zctrl, 'Value', interp1([0 35e-2], [0 1], home(3), 'linear'))
set(handles.thctrl, 'Value', interp1([-pi/2 pi/2], [0 1], home(4), 'linear'))

handles.v.move(home, g)

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
stop(handles.timer), delete(handles.timer)
delete(hObject);


% --- Executes on slider movement.
function yctrl_Callback(hObject, eventdata, handles)
% hObject    handle to yctrl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
move_robot(handles)

% --- Executes during object creation, after setting all properties.
function yctrl_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yctrl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function zctrl_Callback(hObject, eventdata, handles)
% hObject    handle to zctrl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
move_robot(handles)

% --- Executes during object creation, after setting all properties.
function zctrl_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zctrl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function thctrl_Callback(hObject, eventdata, handles)
% hObject    handle to thctrl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
move_robot(handles)

% --- Executes during object creation, after setting all properties.
function thctrl_CreateFcn(hObject, eventdata, handles)
% hObject    handle to thctrl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
