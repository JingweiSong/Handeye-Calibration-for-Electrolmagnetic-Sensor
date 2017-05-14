%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function: Undistort image, select image and generate checkerboard pts in cameara
%             cordinate
%   Method:   
%   Input:    

%   Returns:  
%             data_index:   Index of selcted images
%             Puv_c_0:      Corner points position in camera cordinate
%   Author:   Jingwei Song. 11/05/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;clear all;close all;


num_image = 264;
num_corner = 54;    % 6 by 94
dataset_path     ='D:\WORK\medical SLAM\Ó²¼þ\data\RAWdata\';
bool_image = zeros(num_image,2);    %   col1 - index; col2 - boolean
squareSize = 35/9;         %   Squre size of chkboard
step = 10;              %   Step for fast undistortion

%   Estimate parameters
files      = cell(1, num_image);
files_step = cell(1, floor(num_image/step));
k = 1;
for i = 1:num_image
    files{i} = [dataset_path,int2str(i),'.png'];
    if(mod(i,step)==0 && k<=floor(num_image/step))
        files_step{k} = files{i};
        k = k + 1;
    end
end
[imagePoints, boardSize] = detectCheckerboardPoints(files_step);
worldPoints = generateCheckerboardPoints(boardSize, 4);
cameraParams = estimateCameraParameters(imagePoints, worldPoints);
 
%   Estimate visibility of undistorted images 
hwait=waitbar(0,'Waiting>>>>>>>>');
for i  = 1 : num_image
    filename = [dataset_path,int2str(i),'.png'];
    I = importdata(filename);
    [I,newOrigin] = undistortImage(I,cameraParams,'OutputView','full');
    [imagePoints,boardSize] = detectCheckerboardPoints(I);
    
    %   Judge quality;  Corner number == num_corner or not;
    bool_image(i,1) = i;
    if(size(imagePoints,1) == num_corner)
        bool_image(i,2) = 1;
    else
        bool_image(i,2) = 0;
    end
    
    waitbar(i/num_image,hwait,'Waiting');
%     if(~isempty(imagePoints))
%         figure
%         imshow(I);
%         hold on;
%         plot(imagePoints(:,1),imagePoints(:,2),'ro');
%         title(['figure'  int2str(i)]);
%     end
end
close(hwait);

%   User define selected data
clc
bool_image
num_image = input('Select N by 1 index and store in data_index:');
data_index = zeros(num_image,1);
for i = 1 : num_image
    data_index(i) = input(['The ' int2str(i) 'index:']);
end

%   Gnerate chkboard position in Cam cordinate
Puv_c_0 = zeros(num_corner,3,num_image);
hwait=waitbar(0,'Waiting>>>>>>>>');
for i  = 1 : num_image
    filename = [dataset_path,int2str(data_index(i)),'.png'];
    I = importdata(filename);
    [I,newOrigin] = undistortImage(I,cameraParams,'OutputView','full');
    [imagePoints,boardSize] = detectCheckerboardPoints(I);
    
    if(size(imagePoints,1) ~= num_corner)
        disp('Wrong image, rerun Main_selectimage');
        exit(1);
    end
    
    %   Estimate extrinsic parameter between chkboard cordinate to Cam cordinate
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
    imagePoints = [imagePoints(:,1) + newOrigin(1), ...
                    imagePoints(:,2) + newOrigin(2)];
    [rotationMatrix, translationVector] = extrinsics(...
                    imagePoints,worldPoints,cameraParams);

    %   Estimate corner pts in Cam cordinate
    tmp = zeros(size(worldPoints,1),3);
    tmp(:,1:2) = worldPoints;
    worldPoints = tmp; clear tmp;
    worldPoints = worldPoints * rotationMatrix + repmat(translationVector,size(worldPoints,1),1);
         
    Puv_c_0(:,:,i) = worldPoints;
    waitbar(i/num_image,hwait,'Waiting');
    
%     figure
%     plotCamera('Location',[0 0 0],'Orientation',eye(3,3),'Size',10);
%     hold on
%     pcshow(worldPoints,'MarkerSize',10);
%     hold on
%     pcshow(worldPoints(1,:),'MarkerSize',100);
end
close(hwait);

save data_index data_index;
save Puv_c_0 Puv_c_0;