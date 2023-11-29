
function varargout = Baot_Ferdig(varargin)
% BAOT_FERDIG MATLAB code for Baot_Ferdig.fig
%      BAOT_FERDIG, by itself, creates a new BAOT_FERDIG or raises the existing
%      singleton*.
%
%      H = BAOT_FERDIG returns the handle to a new BAOT_FERDIG or the handle to
%      the existing singleton*.
%
%      BAOT_FERDIG('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in BAOT_FERDIG.M with the given input arguments.
%
%      BAOT_FERDIG('Property','Value',...) creates a new BAOT_FERDIG or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Baot_Ferdig_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Baot_Ferdig_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Baot_Ferdig

% Last Modified by GUIDE v2.5 15-Nov-2023 14:34:11

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Baot_Ferdig_OpeningFcn, ...
                   'gui_OutputFcn',  @Baot_Ferdig_OutputFcn, ...
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


% --- Executes just before Baot_Ferdig is made visible.
function Baot_Ferdig_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Baot_Ferdig (see VARARGIN)

% Choose default command line output for Baot_Ferdig
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Baot_Ferdig wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Baot_Ferdig_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function theta_1_Callback(hObject, eventdata, handles)
% hObject    handle to theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta_1 as text
%        str2double(get(hObject,'String')) returns contents of theta_1 as a double


% --- Executes during object creation, after setting all properties.
function theta_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta_2_Callback(hObject, eventdata, handles)
% hObject    handle to theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta_2 as text
%        str2double(get(hObject,'String')) returns contents of theta_2 as a double


% --- Executes during object creation, after setting all properties.
function theta_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta_3_Callback(hObject, eventdata, handles)
% hObject    handle to theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta_3 as text
%        str2double(get(hObject,'String')) returns contents of theta_3 as a double


% --- Executes during object creation, after setting all properties.
function theta_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta_4_Callback(hObject, eventdata, handles)
% hObject    handle to theta_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta_4 as text
%        str2double(get(hObject,'String')) returns contents of theta_4 as a double


% --- Executes during object creation, after setting all properties.
function theta_4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in forward.
function forward_Callback(hObject, eventdata, handles)
% hObject    handle to forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

syms th0 th1 th3 th4

% FERDIG BAOT ARM
% Laga av Vegard Ullebø, Roar Bøyum og Peter Skaar


%% BOX PLOT

coor_Scope = [ 47.5,  52.5;   %y
               -22.5 , -17.5;   %z
               -12.5,  -7.5;]; %x
A=[coor_Scope(1,1),coor_Scope(2,2),coor_Scope(3,2);
   coor_Scope(1,1),coor_Scope(2,2),coor_Scope(3,1);
   coor_Scope(1,2),coor_Scope(2,2),coor_Scope(3,1);
   coor_Scope(1,2),coor_Scope(2,2),coor_Scope(3,2);
   coor_Scope(1,1),coor_Scope(2,1),coor_Scope(3,2);
   coor_Scope(1,1),coor_Scope(2,1),coor_Scope(3,1);
   coor_Scope(1,2),coor_Scope(2,1),coor_Scope(3,1);
   coor_Scope(1,2),coor_Scope(2,1),coor_Scope(3,2)];
d=[1 2 3 4 8 5 6 7 3 2 6 5 1 4 8 7];
X = A(d,3);
Y = A(d,1);
Z = A(d,2);



Th_1 = str2double(char(get(handles.theta_1,'String')))*pi/180;
Th_2 = str2double(char(get(handles.theta_2,'String')))*pi/180;
Th_3 = str2double(char(get(handles.theta_3,'String')))*pi/180;
Th_4 = str2double(char(get(handles.theta_4,'String')))*pi/180;

%Vår robot
L_1 = 5;
L_2 = 34.4;
L_3 = 21.26;
L_4 = 15.7;
L_5 = 5;

% Our DH paramaters
%DH = [L_1 L_2 L_3 L_4 L_5;  L_1 0 0 0 0; pi/2 0 0 pi/2 pi;  ]'

DH = [0 L_2 L_3 L_4 0;    %a
      L_1 0 0 0 L_5;            % d
       pi/2 0 0 pi/2 0;           % aplha
        th0' th1' th3' th4' pi/2]' %theta

j1 = Revolute('d', L_1, 'a', 0,   'alpha', pi/2);
j2 = Revolute('d', 0,   'a', L_2, 'alpha', 0);
j3 = Revolute('d', 0,   'a', L_3, 'alpha', 0);
j4 = Revolute('d', 0,   'a', L_4, 'alpha', pi/2);
j5 = Revolute('d', L_5,   'a', 0, 'alpha', 0);


Robot = SerialLink([j1 j2 j3 j4 j5],'name', 'my robot');
Robot.name = 'Baot_arm';
Robot.qlim = [-180, 180; 1, 90; -90, 90; -90, 90; 0, 0]*pi/180;

global angle_offset theta_initial th1_prev th2_prev th3_prev th4_prev P
angle_offset = [th1_prev th2_prev th3_prev th4_prev];
theta_initial = [th1_prev, th2_prev, th3_prev th4_prev]

th1_prev = Th_1;
th2_prev = Th_2;
th3_prev = Th_3;
th4_prev = Th_4;


Th_path{1} = linspace(theta_initial(1),Th_1,      20);
Th_path{2} = linspace(theta_initial(2),Th_2,      20);
Th_path{3} = linspace(theta_initial(3),Th_3,      20);
Th_path{4} = linspace(theta_initial(4),Th_4,      20);

% Initial posision

P_path=[];
for i=1:1:20    
P_path = [P_path fkine4DOF(DH, [Th_path{1}(i) Th_path{2}(i) Th_path{3}(i) Th_path{4}(i)])];
end

%%

plot3(P_path(1,:), P_path(2,:),P_path(3,:),'Color', [1 0 0],'LineWidth',2);
hold on;
plot3(A(d,3),A(d,1),A(d,2));
hold on;
patch([X(1:6) flip(X(1:6))], [Y(1:6) flip(Y(1:6))], [Z(1:6) flip(Z(1:6))], 'r', 'FaceAlpha',0.25)
kp = 2;
patch([X((1:6)+kp) flip(X((1:6)+kp))], [Y((1:6)+kp) flip(Y((1:6)+kp))], [Z((1:6)+kp) flip(Z((1:6)+kp))], 'g', 'FaceAlpha',0.25)
kp = 10;
patch([X((1:6)+kp) flip(X((1:6)+kp))], [Y((1:6)+kp) flip(Y((1:6)+kp))], [Z((1:6)+kp) flip(Z((1:6)+kp))], 'b', 'FaceAlpha',0.25)
hold on;

xlim([-80 80])
ylim([-80 80])
zlim([-80 80])
for i=1:20
Robot.plot([Th_path{1}(i) Th_path{2}(i) Th_path{3}(i) Th_path{4}(i) pi/2],'scale',0.5)
end
hold off;
angle_start = [0 0 0 0]
P = fkine4DOF(DH, [Th_1 Th_2 Th_3 Th_4], angle_start)
P = eval(P)


set(handles.Pos_X, 'string', num2str(round(P(1),3)))
set(handles.Pos_Y, 'string', num2str(round(P(2),3)))
set(handles.Pos_Z, 'string', num2str(round(P(3),3)))

function Pos_X_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_X as text
%        str2double(get(hObject,'String')) returns contents of Pos_X as a double


% --- Executes during object creation, after setting all properties.
function Pos_X_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Pos_Y_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_Y as text
%        str2double(get(hObject,'String')) returns contents of Pos_Y as a double


% --- Executes during object creation, after setting all properties.
function Pos_Y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Pos_Z_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_Z as text
%        str2double(get(hObject,'String')) returns contents of Pos_Z as a double


% --- Executes during object creation, after setting all properties.
function Pos_Z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Invers.
function Invers_Callback(hObject, eventdata, handles)
% hObject    handle to Invers (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

syms th0 th1 th3 th4

PX = str2double(char(get(handles.Pos_X,'String')));
PY = str2double(char(get(handles.Pos_Y,'String')));
PZ = str2double(char(get(handles.Pos_Z,'String')));

L_1 = 5;
L_2 = 34.4;
L_3 = 21.26;
L_4 = 15.7;
L_5 = 5;

j1 = Revolute('d', L_1, 'a', 0,   'alpha', pi/2);
j2 = Revolute('d', 0,   'a', L_2, 'alpha', 0);
j3 = Revolute('d', 0,   'a', L_3, 'alpha', 0);
j4 = Revolute('d', 0,   'a', L_4, 'alpha', pi/2);
j5 = Revolute('d', L_5,   'a', 0, 'alpha', 0);

Robot = SerialLink([j1 j2 j3 j4 j5]);
Robot.name = 'Baot'

T = [   1 0 0 PX; 
        0 1 0 PY; 
        0 0 1 PZ;
        0 0 0 1 ];

xlim([-80 80])
ylim([-80 80])
zlim([-80 80])

J = Robot.ikine(T,[0 0 0 0 0],'mask',[1 1 1 0 0 0])*180/pi

set(handles.theta_1, 'string', J(1));
set(handles.theta_2, 'string', J(2));
set(handles.theta_3, 'string', J(3)); 
set(handles.theta_4, 'string', J(4));
