%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function: Implement handeye calibration
%   Method:   See the document
%   Input:    
%             Num_frame:    Number of frames
%             Num_corner:   Number of corner points
%             Re_g:         Rotation of EM sensor (GlobalPos). 3*3
%             Te_g:         Translation of EM sensor (GlobalPos).3*1
%             Puv_g:        Position of each corner.(GlobalPos).3*Num_corner
%             Puv_c_0:     Position of each corner.(CameraPos).3*Num_corner
%             squareSize:   Size of the square in chekerboard
%             cameraParams: Camera parameter
%   Returns:  
%             Rc_e:         Rotation from EM sensor to camera
%             Tc_e:         Translation from EM sensor to camera
%   Author:   Jingwei Song.   23/04/2017 to ...
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;clear all;close all;

DEBUG = 0;

% =============== Parameter settering =================== %
dataset_path     ='D:\WORK\medical SLAM\Ó²¼ş\data\';
load data_index;
load chkBoard.mat
load Puv_c_0.mat
Num_frame  = size(data_index,1);
num_row    = 6;
num_col    = 9;
Num_corner = 54;    %   6 by 9
squareSize = 35/9;    %   Square size of checkerboard (in millimeters)
% ------------------------------------------------------- %

% =============== Variable initilization =================== %
% Puv_c_0 = zeros(Num_corner,3,Num_frame);
M       = zeros(Num_corner,3,Num_frame);
%chkBoard= zeros(Num_corner,3);

% ------------------------------------------------------- %

% ===================== Read Data ==========================%
% files = cell(1, Num_frame);
%  for i = 1:Num_frame
%     files{i} = [dataset_path,int2str(data_index(i)),'.png'];
%  end

%   Get corners in global
% ------------ debug --------test orientation1--------  %
chkBoard1 = zeros(size(chkBoard));
for i = 1 : 9
    for j = 1 : 6
        chkBoard1((i-1)*6+j,:) = chkBoard(i*6-(j-1),:);
    end
end
chkBoard = chkBoard1;clear chkBoard1;
% ------------ end debug ----------------------------  %
 tmp = chkBoard(:,2:4);
 [chkBoard error] = getGlobalPosCheckerboard(tmp,num_row,num_col,squareSize); %   Convert measured
 disp('Overall error of checkboard estimation (mm):')
 error


%   Get emsensor Rotation and translation
emsensor=load([dataset_path 'pose.txt']);
 Re_g = zeros(3,3,Num_frame);
 Te_g = zeros(Num_frame,3);
 for i = 1 : Num_frame
      Re_g(:,:,i) = quat2rotm(emsensor(data_index(i),4:7));
     %Re_g(:,:,i) = quat2rotm([emsensor(data_index(i),7) emsensor(data_index(i),4:6)]);
     Te_g(i,:)   = emsensor(data_index(i),1:3);
 end
% ---------------------------------------------------------%


% % ======== Camera Calibration and corner points ====================%
% [imagePoints, boardSize] = detectCheckerboardPoints(files);
% worldPoints = generateCheckerboardPoints(boardSize, squareSize);
% cameraParams = estimateCameraParameters(imagePoints, worldPoints);
% for i = 1:Num_frame
%     filename = [dataset_path,int2str(data_index(i)),'.png'];
%     imOrig = importdata(filename);
%     Puv_c_0(:,:,i) = getCheckboardPos_camera( imOrig,squareSize,cameraParams,Num_corner);
% end
% % ----------------------------------------------------------%

%%  Optimization Rc_e and Tc_e
%   Initialize
x = zeros(6,1);
Rc_e = eye(3,3);
%   Calculate M(Num_corner,3,Num_frame)
for i = 1:Num_frame
    for j = 1:Num_corner
%         M(j,:,i) = Re_g(:,:,i)'*chkBoard(j,:)' - Te_g(i,:)';
        M(j,:,i) = Re_g(:,:,i)'* (chkBoard(j,:)' - Te_g(i,:)');
        %M(j,:,i) = Re_g(:,:,i)'* (chkBoard(j,:)' - Te_g(i,:)');
    end
end

%  #define
if(DEBUG == 1)
    figure
    plotCamera('Location',[0 0 0],'Orientation',eye(3,3),'Size',5);
    hold on
    pcshow(M(:,:,1), ...
        'VerticalAxisDir','down','MarkerSize',30);
    pcshow(Puv_c_0(:,:,1), ...
        'VerticalAxisDir','down','MarkerSize',30);  
    hold on
    pcshow(M(1,:,1),'MarkerSize',100);
    hold on
    pcshow(Puv_c_0(1,:,1),'MarkerSize',100);
end


F = CalculateF( Rc_e,x,M,Puv_c_0,Num_frame,Num_corner);
J = CalculateJ( Rc_e,x,M,Puv_c_0,Num_frame,Num_corner );
P = diag(ones(size(F,1),1));
min_FX_old = 10000000000000;
k=0;
I = eye(size(J,2));
threshold = 0.00000000000001;
while ((F'*P*F)>threshold&&abs(F'*P*F-min_FX_old)>threshold&&k<100)
    min_FX_old = F'*P*F
    J = sparse(J);  
    F = sparse(F);
    
    u = 0.0001;
%     d = -(J'*P*J+u*I)\(J'*P*F);
    d = -(J'*P*J)\(J'*P*F);
    x_new = x + d;
    Rc_e_new = Rc_e*expm(skew(d(1:3)));
    F_new = CalculateF(Rc_e_new,x_new,M,Puv_c_0,Num_frame,Num_corner);
    min_FX = F_new'*P*F_new;
    t = 1;
    while(min_FX > min_FX_old&&t<10)
        % LM algorithm
        d = -(J'*P*J+u*I)\(J'*P*F);
        d = -(J'*P*J)\(J'*P*F);
        x_new = x + d;
        Rc_e_new = Rc_e*expm(skew(d(1:3)));
        F_new = CalculateF(Rc_e_new,x_new,M,Puv_c_0,Num_frame,Num_corner);
        min_FX = F_new'*P*F_new;
        u  = u * 10;
        t =  t + 1;
    end
    if(t==10)
        break
    end
    x = x + d;
    Rc_e = Rc_e*expm(skew(d(1:3)));
    k=k+1;
    F = CalculateF( Rc_e,x,M,Puv_c_0,Num_frame,Num_corner);
    J = CalculateJ( Rc_e,x,M,Puv_c_0,Num_frame,Num_corner );
end

Tc_e = x(4:6);
error = 0;
for i = 1 : Num_frame
    for j = 1 : Num_corner
        tmp = Rc_e*M(j,:,i)'+x(4:6)-Puv_c_0(j,:,i)';
        error = error + sqrt(tmp'*tmp);
    end
end
error = error / (Num_corner*Num_frame);
Rc_e
x(4:6)
disp('Overall error of checkboard estimation (mm):')
error