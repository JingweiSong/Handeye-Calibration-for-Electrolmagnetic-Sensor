%   基本思路是多付相片估计camera parameter(内参焦距畸变等),然后用最后一副估计外参

%% Compute Extrinsics
% Load calibration images.

% Copyright 2015 The MathWorks, Inc.

numImages = 9;
files = cell(1, numImages);
 for i = 1:numImages
    files{i} = fullfile(matlabroot, 'toolbox', 'vision', 'visiondata', 'calibration', 'slr', sprintf('image%d.jpg', i));
 end

%%
% Detect the checkerboard corners in the images.
[imagePoints, boardSize] = detectCheckerboardPoints(files);
 
%% 
% Generate the world coordinates of the checkerboard corners in the pattern-centric coordinate system, with the upper-left corner at (0,0).
squareSize = 29; % in millimeters
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
 
%%
% Calibrate the camera.
cameraParams = estimateCameraParameters(imagePoints, worldPoints);
 
%% 
% Load image at new location.
imOrig = imread(fullfile(matlabroot, 'toolbox', 'vision', 'visiondata', 'calibration', 'slr', 'image9.jpg'));
figure; imshow(imOrig);
title('Input Image');
 
%% 
% Undistort image.
im = undistortImage(imOrig, cameraParams);
 
%% 
% Find reference object in new image.
[imagePoints, boardSize] = detectCheckerboardPoints(im);
hold on;
plot(imagePoints(:,1),imagePoints(:,2),'ro');
 
%% 
% Compute new extrinsics.
[rotationMatrix, translationVector] = extrinsics(imagePoints, worldPoints, cameraParams)
%%Undistort image.

[im,newOrigin] = undistortImage(imOrig,cameraParams,'OutputView','full');
%%Find reference object in new image.

[imagePoints,boardSize] = detectCheckerboardPoints(im);
%%Compensate for image coordinate system shift.

imagePoints = [imagePoints(:,1) + newOrigin(1), ...
             imagePoints(:,2) + newOrigin(2)];
%%Compute new extrinsics.

[rotationMatrix, translationVector] = extrinsics(...
imagePoints,worldPoints,cameraParams);
%%Compute camera pose.

[orientation, location] = extrinsicsToCameraPose(rotationMatrix, ...
  translationVector);
figure
plotCamera('Location',location,'Orientation',orientation,'Size',20);
hold on
pcshow([worldPoints,zeros(size(worldPoints,1),1)], ...
  'VerticalAxisDir','down','MarkerSize',40);