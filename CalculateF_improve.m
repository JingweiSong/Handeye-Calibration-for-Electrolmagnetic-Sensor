function [ F ] = CalculateF_improve(Rc_e,x,Re_g,Te_g,Puv_c_0,Num_frame,Num_corner,chkBoard )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function: Calculate objective function
%   Method:   See the document
%   Input:    
%             Rc_e:         Rotation from EM sensor to camera (in smooth
%                           manifold)
%             x(4:6):       Translation from EM sensor to camera
%             Re_g:         Rotation matrix of EMsensor (3,3,Num_frame)
%             Te_g:         Translation vector of EMsensor (3,Num_frame)
%             Puv_c_0:      Checkerboard pos in camera cordinate    (Num_corner,3,Num_frame)
%             Num_frame:    Number of frames
%             Num_corner:   Number of corner points
%             chkBoard:     Corners of checkerboard
%             Num_corner:   Number of corner points
%   Returns:  
%             F;            Object function
%   Author:   Jingwei Song.   13/05/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
F = zeros(3*Num_frame*Num_corner+6*Num_frame,1);

M     = zeros(Num_corner,3,Num_frame);
for i = 1:Num_frame
    for j = 1:Num_corner
        d = x(6*(i-1)+1:6*(i-1)+3);
        M(j,:,i) = Re_g(:,:,i)'* expm(skew(d))* (chkBoard(j,:)' - Te_g(i,:)' + x(6*(i-1)+4:6*(i-1)+6));
    end
end

%   PartI: Rc_e*Mij+Tc_e
for i = 1 : Num_frame
    for j = 1 : Num_corner
        id = (i-1)*Num_corner+j;
        F(3*(id-1)+1:3*id) = Rc_e*M(j,:,i)'+x(end-2:end)-Puv_c_0(j,:,i)';
    end
end

%   PartII & III: xi ti
start = 3*Num_frame*Num_corner;
for i = 1 : Num_frame
    F(start+6*(i-1)+1:start+6*(i-1)+6) = x(6*(i-1)+1:6*(i-1)+6);
end

end

