function varargout = untitled(varargin)
% UNTITLED MATLAB code for untitled.fig
%      UNTITLED, by itself, creates a new UNTITLED or raises the existing
%      singleton*.
%
%      H = UNTITLED returns the handle to a new UNTITLED or the handle to
%      the existing singleton*.
%
%      UNTITLED('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in UNTITLED.M with the given input arguments.
%
%      UNTITLED('Property','Value',...) creates a new UNTITLED or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before untitled_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to untitled_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help untitled

% Last Modified by GUIDE v2.5 12-Jun-2016 15:42:00

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @untitled_OpeningFcn, ...
                   'gui_OutputFcn',  @untitled_OutputFcn, ...
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


% --- Executes just before untitled is made visible.
function untitled_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to untitled (see VARARGIN)
set(hObject,'Toolbar','figure'); 

% Choose default command line output for untitled
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes untitled wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = untitled_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%根据标准D-H参数建立机械臂link([Alpha A theta d],'standard')
cla reset;
L1 = link([1/2*pi 4 0 0],'standard');
L2 = link([1/2*pi 3 0 0],'standard');
L3 = link([0 2 0 0],'standard');
bot=robot({L1 L2 L3},'bot');
%bot2=bot;
%axes(handles.axes2);
%drivebot1(bot2);
%qz为机械臂的初始关节角
qz=[0 0 0];
%qr为机械臂目标关节角
qr=[1/3*pi 1/2*pi 1*pi];
%将时间分切，规划机械臂运动路径
t=0:0.009:1;
[q,qd,qdd]=jtraj(qz,qr,t);%q为转过的角度，qd为机械臂速度，qdd为加速度
axes(handles.axes6);plot(q);xlabel('Time');ylabel('q');
axes(handles.axes7);plot(qd);xlabel('Time');ylabel('qd');
axes(handles.axes8);plot(qdd);xlabel('Time');ylabel('qdd');
T=fkine(bot,qr);   %前向运动学求出变换矩阵
qi=ikine(bot,T);  %逆运动学求出每个关节角转的角度
%求出机械臂末端轨迹
m=fkine(bot,q);
for i=1:1:length(t)
    n=m(:,:,i);
    x(1,i)=n(1,4);
    y(1,i)=n(2,4);
    z(1,i)=n(3,4);
end
%绘制末端轨迹散点图
axes(handles.axes5);
scatter3(x,y,z);xlabel('X');ylabel('Y');zlabel('Z');
%绘制末端轨迹连线三维图
axes(handles.axes1);
boxplot3(n(1,4)-0.15,n(2,4)-0.15,n(3,4)-0.15,0.3,0.3,0.3);axis equal
hold on
axes(handles.axes1);
plot3(x,y,z);xlabel('X');ylabel('Y');zlabel('Z');
grid on;
%绘制机械臂运动循环动图
set(handles.text4,'String',num2str(qr));
for i=1:1:4
    p1(1,i)=T(1,i);
    p2(1,i)=T(2,i);
    p3(1,i)=T(3,i);
    p4(1,i)=T(4,i);
end
set(handles.text7,'String',num2str(p1));
set(handles.text8,'String',num2str(p2));
set(handles.text9,'String',num2str(p3));
set(handles.text10,'String',num2str(p4));
guidata(hObject,handles);
axes(handles.axes1);
plot(bot,q);



% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla reset;
a=get(handles.edit1,'String');
T = str2num(char(a));
L1 = link([1/2*pi 4 0 0],'standard');
L2 = link([1/2*pi 3 0 0],'standard');
L3 = link([0 2 0 0],'standard');
bot2=robot({L1 L2 L3},'bot');
t=0:0.009:1;
qz=[0 0 0];
qr=ikine(bot2,T);
[q,qd,qdd]=jtraj(qz,qr,t);
axes(handles.axes6);plot(q);xlabel('Time');ylabel('q');
axes(handles.axes7);plot(qd);xlabel('Time');ylabel('qd');
axes(handles.axes8);plot(qdd);xlabel('Time');ylabel('qdd');
m=fkine(bot2,q);
for i=1:1:length(t)
    n=m(:,:,i);
    x(1,i)=n(1,4);
    y(1,i)=n(2,4);
    z(1,i)=n(3,4);
end
axes(handles.axes5);
scatter3(x,y,z);xlabel('X');ylabel('Y');zlabel('Z');
%绘制末端轨迹连线三维图
axes(handles.axes1);
boxplot3(n(1,4)-0.15,n(2,4)-0.15,n(3,4)-0.15,0.3,0.3,0.3);axis equal
hold on
plot3(x,y,z);xlabel('X');ylabel('Y');zlabel('Z');
grid on;
set(handles.text4,'String',num2str(qr));
for i=1:1:4
    p1(1,i)=T(1,i);
    p2(1,i)=T(2,i);
    p3(1,i)=T(3,i);
    p4(1,i)=T(4,i);
end
set(handles.text7,'String',num2str(p1));
set(handles.text8,'String',num2str(p2));
set(handles.text9,'String',num2str(p3));
set(handles.text10,'String',num2str(p4));
guidata(hObject,handles);
plot(bot2,q)



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%根据标准D-H参数建立机械臂link([Alpha A theta d],'standard')
L1 = link([1/2*pi 4 0 0],'standard');
L2 = link([1/2*pi 3 0 0],'standard');
L3 = link([0 2 0 0],'standard');
bot2=robot({L1 L2 L3},'bot');
drivebot(bot2);
