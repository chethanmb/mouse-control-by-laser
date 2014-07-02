function varargout = laser(varargin)
% LASER MATLAB code for laser.fig
%      LASER, by itself, creates a new LASER or raises the existing
%      singleton*.
%
%      H = LASER returns the handle to a new LASER or the handle to
%      the existing singleton*.
%
%      LASER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LASER.M with the given input arguments.
%
%      LASER('Property','Value',...) creates a new LASER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before laser_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to laser_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help laser

% Last Modified by GUIDE v2.5 10-Jun-2014 09:51:57

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @laser_OpeningFcn, ...
                   'gui_OutputFcn',  @laser_OutputFcn, ...
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


% --- Executes just before laser is made visible.
function laser_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to laser (see VARARGIN)

% Choose default command line output for laser
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes laser wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = laser_OutputFcn(hObject, eventdata, handles) 
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
clc;
clear all;
close all;


redThresh = 0.24; % Threshold for red detection
greenThresh=0.06;


blueThresh = 0.15; % Threshold for blue detection

FlagG=1;
FlagClick=0;
FlagDrag=1;
CentroidX=[];
CentroidY=[];

DragSen=10;

count=0;

vidDevice = imaq.VideoDevice('winvideo', 1, 'YUY2_640x480', ... 
                    'ROI', [1 1 640 480], ...
                    'ReturnedColorSpace', 'rgb');

%set(vidDevice.DeviceProperties, 'Brightness', 0); 

set(vidDevice.DeviceProperties, 'Exposure', -7); 

set(vidDevice.DeviceProperties, 'FrameRate', '30.0000');
                
set(vidDevice.DeviceProperties, 'Sharpness', 100); 

set(vidDevice.DeviceProperties, 'Zoom',2);


vidInfo = imaqhwinfo(vidDevice);
  
hblob = vision.BlobAnalysis('AreaOutputPort', false, ... 
                                'CentroidOutputPort', true, ... 
                                'BoundingBoxOutputPort', true', ...
                                'MinimumBlobArea', 600, ...
                                'MaximumBlobArea', 3000, ...
                                'MaximumCount', 1);

 hshapeinsBox = vision.ShapeInserter('BorderColorSource', 'Input port', ... % box handling
                                         'Fill', true, ...
                                         'FillColorSource', 'Input port', ...
                                         'Opacity', 0.2);


htextinsCent = vision.TextInserter('Text', '+ X:%d,Y:%d', ... % text for centroid
                                    'LocationSource', 'Input port', ...
                                    'Color', [1 1 1], ... 
                                    'Font', 'Courier New', ...
                                    'FontSize', 15);
                                

hVideoIn = vision.VideoPlayer('Name', 'Final Video', ... % video player
                                'Position', [400 100 vidInfo.MaxWidth+20 vidInfo.MaxHeight+30]);


%nFrame = 0;

import java.awt.Robot;
import java.awt.event.*;

cursor = Robot;

while(1)
    
    rgbFrame = step(vidDevice); 

    diffFrameGreen = imsubtract(rgbFrame(:,:,2), rgb2gray(rgbFrame)); % Get green component of the image
    diffFrameGreen = medfilt2(diffFrameGreen, [3 3]); % Filter out the noise by using median filter
    binFrameGreen = im2bw(diffFrameGreen, greenThresh); % Convert the image into binary image with the green objects as white
    %binFrameGreen = bwareaopen(binFrameGreen,500);
     
     
   [centroidGreen, bboxGreen] = step(hblob, binFrameGreen); % Get the centroids and bounding boxes of the green blobs
   centroidGreen = uint16(centroidGreen); % Convert the centroids into Integer for further steps  

   vidIn = step(hshapeinsBox, rgbFrame, bboxGreen, single([0 1 0])); % Insert the green box
   
   
   
   
   
   for object = 1:1:length(bboxGreen(:,1)) % Write the corresponding centroids for green
        
        centXGreen = centroidGreen(object,1);
        centYGreen = centroidGreen(object,2);
        vidIn = step(htextinsCent, vidIn, [centXGreen centYGreen], [centXGreen-6 centYGreen-9]); 
   
        cursor.mouseMove(2.14*centXGreen,1.6*centYGreen);
   
        
             
        count=count+1;
        
        if (count>50)
                             
                   display('double click');
                   cursor.mousePress(InputEvent.BUTTON1_MASK);
                   
                   cursor.mouseRelease(InputEvent.BUTTON1_MASK);
                  
                   cursor.mousePress(InputEvent.BUTTON1_MASK);
                   
                   cursor.mouseRelease(InputEvent.BUTTON1_MASK);
       count=0;
        
        end     
   end

   
   step(hVideoIn, vidIn); % Output video stream
   
    %nFrame = nFrame+1;
    
   
end
   
release(hVideoIn);
release(vidDevice);   

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clc;
clear all;
close all;

greenThresh=0.06;

FlagG=1;
FlagClick=0;
FlagDrag=1;
CentroidX=[];
CentroidY=[];

DragSen=10;

count=0;

vidDevice = imaq.VideoDevice('winvideo', 1, 'YUY2_640x480', ... 
                    'ROI', [1 1 640 480], ...
                    'ReturnedColorSpace', 'rgb');

%set(vidDevice.DeviceProperties, 'Brightness', 0); 

set(vidDevice.DeviceProperties, 'Exposure', -7); 

set(vidDevice.DeviceProperties, 'FrameRate', '30.0000');
                
set(vidDevice.DeviceProperties, 'Sharpness', 100); 

set(vidDevice.DeviceProperties, 'Zoom',2);


vidInfo = imaqhwinfo(vidDevice);
  
hblob = vision.BlobAnalysis('AreaOutputPort', false, ... 
                                'CentroidOutputPort', true, ... 
                                'BoundingBoxOutputPort', true', ...
                                'MinimumBlobArea', 600, ...
                                'MaximumBlobArea', 3000, ...
                                'MaximumCount', 1);

 hshapeinsBox = vision.ShapeInserter('BorderColorSource', 'Input port', ... % box handling
                                         'Fill', true, ...
                                         'FillColorSource', 'Input port', ...
                                         'Opacity', 0.2);


htextinsCent = vision.TextInserter('Text', '+ X:%d,Y:%d', ... % text for centroid
                                    'LocationSource', 'Input port', ...
                                    'Color', [1 1 1], ... 
                                    'Font', 'Courier New', ...
                                    'FontSize', 15);
                                

hVideoIn = vision.VideoPlayer('Name', 'Final Video', ... % video player
                                'Position', [400 100 vidInfo.MaxWidth+20 vidInfo.MaxHeight+30]);


%nFrame = 0;

import java.awt.Robot;
import java.awt.event.*;

cursor = Robot;

while(1)
    
    rgbFrame = step(vidDevice); 

    diffFrameGreen = imsubtract(rgbFrame(:,:,2), rgb2gray(rgbFrame)); % Get green component of the image
    diffFrameGreen = medfilt2(diffFrameGreen, [3 3]); % Filter out the noise by using median filter
    binFrameGreen = im2bw(diffFrameGreen, greenThresh); % Convert the image into binary image with the green objects as white
    %binFrameGreen = bwareaopen(binFrameGreen,500);
     
     
   [centroidGreen, bboxGreen] = step(hblob, binFrameGreen); % Get the centroids and bounding boxes of the green blobs
   centroidGreen = uint16(centroidGreen); % Convert the centroids into Integer for further steps  

   vidIn = step(hshapeinsBox, rgbFrame, bboxGreen, single([0 1 0])); % Insert the green box
   
   
   
   
   
   for object = 1:1:length(bboxGreen(:,1)) % Write the corresponding centroids for green
        
        centXGreen = centroidGreen(object,1);
        centYGreen = centroidGreen(object,2);
        vidIn = step(htextinsCent, vidIn, [centXGreen centYGreen], [centXGreen-6 centYGreen-9]); 
   
        cursor.mouseMove(2.14*centXGreen,1.6*centYGreen);
   
        
       
        
        if(centXGreen>500&&centYGreen>400)
           
            count=count+1;
            
            if(count>20)
            
                display('Scroll down');
                cursor.mouseWheel(+1);
                pause(1);
            count=0;
            end
            
        end 
       
        if(centXGreen>500&&centYGreen<100)
            
            count=count+1;
            
            if(count>20)
            
                display('Scroll up');
                cursor.mouseWheel(-1);
                pause(1);
             count=0;
            end
           
        end
        
         
   end

   
   step(hVideoIn, vidIn); % Output video stream
   
    %nFrame = nFrame+1;
    
   
end
   
release(hVideoIn);
release(vidDevice);   


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear all;
close all;
