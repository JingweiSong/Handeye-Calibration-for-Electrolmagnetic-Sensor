function [ worldPoints ] = getCheckboardPos_camera( image,squareSize,cameraParams,num_corner )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function: getCheckboardPos_camera
%   Method:   Get the checkboard corner position in camera coordinate
%   Input:    image:        Checkerboard image
%             squareSize:   Size of the square in chekerboard
%             cameraParams: Camera parameter
%             num_corner:   Number of corners
%   Returns:  World coordinate of 
%   Author:   Jingwei Song.   23/04/2017 to ...
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[im,newOrigin] = undistortImage(image,cameraParams,'OutputView','full');
[imagePoints,boardSize] = detectCheckerboardPoints(im);
if(size(imagePoints,1) ~= num_corner)
    if(~isempty(imagePoints))
        figure
        imshow(im);
        hold on;
        plot(imagePoints(:,1),imagePoints(:,2),'ro');
        title(['figure'  int2str(i)]);
    end
    disp('Wrong image, rerun Main_selectimage');
    exit(1);
end
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
imagePoints = [imagePoints(:,1) + newOrigin(1), ...
             imagePoints(:,2) + newOrigin(2)];
% if(DEBUG == 1)
%     figure; imshow(image);
%     title('Input Image');
%     hold on;
%     plot(imagePoints(:,1),imagePoints(:,2),'ro');
% end
[rotationMatrix, translationVector] = extrinsics(...
imagePoints,worldPoints,cameraParams);

        
% %   #define
% if(DEBUG == 1)
% [orientation, location] = extrinsicsToCameraPose(rotationMatrix, ...
%             translationVector);
%     figure
%     plotCamera('Location',location,'Orientation',orientation,'Size',20);
%     hold on
%     pcshow([worldPoints,zeros(size(worldPoints,1),1)], ...
%         'VerticalAxisDir','down','MarkerSize',40);
% end

tmp = zeros(size(worldPoints,1),3);
tmp(:,1:2) = worldPoints;
worldPoints = tmp; clear tmp;
worldPoints = worldPoints * rotationMatrix + repmat(translationVector,size(worldPoints,1),1);

end

