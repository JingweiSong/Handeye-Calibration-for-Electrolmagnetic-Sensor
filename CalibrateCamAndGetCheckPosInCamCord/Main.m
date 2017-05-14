clc;clear all;close all;
DEBUG = 1;

%   Load data (In the form of pathtodata0
numImages = 9;
files = cell(1, numImages);
 for i = 1:numImages
    files{i} = fullfile(matlabroot, 'toolbox', 'vision', 'visiondata', 'calibration', 'slr', sprintf('image%d.jpg', i));
 end

%   (I)  --  Calibrate Camera  ||   get camera parameters
[imagePoints, boardSize] = detectCheckerboardPoints(files);
squareSize = 29; % in millimeters
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
cameraParams = estimateCameraParameters(imagePoints, worldPoints);

%   (II) --  Get camera extrinsic parameters
imOrig = imread(fullfile(matlabroot, 'toolbox', 'vision', 'visiondata', 'calibration', 'slr', 'image9.jpg'));
worldPoints = getCheckboardPos_camera( imOrig,squareSize,cameraParams );
if(DEBUG == 1)
    figure
    plotCamera('Location',[0 0 0],'Orientation',eye(3,3),'Size',20);
    hold on
    pcshow(worldPoints, ...
        'VerticalAxisDir','down','MarkerSize',40);
end