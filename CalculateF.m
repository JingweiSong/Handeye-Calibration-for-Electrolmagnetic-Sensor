function [ F ] = CalculateF(Rc_e,x,M,Puv_c_0,Num_frame,Num_corner )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function: Calculate objective function
%   Method:   See the document
%   Input:    
%             Rc_e:       Rotation from EM sensor to camera (in smooth
%                           manifold)
%             x(4:6):       Translation from EM sensor to camera
%             M:         	Checkerboard pos in EM sensor cordinate (Num_corner,3,Num_frame)
%             Puv_c_0:      Checkerboard pos in camera cordinate    (Num_corner,3,Num_frame)
%             Num_frame:    Number of frames
%             Num_corner:   Number of corner points
%   Returns:  
%             F;            Object function
%   Author:   Jingwei Song.   09/05/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
F = zeros(3*Num_frame*Num_corner,1);
for i = 1 : Num_frame
    for j = 1 : Num_corner
        %F((i-1)*Num_corner+j) = M(j,:,i)*Rc_e+x(4:6)-Puv_c_0(j,:,i);
        id = (i-1)*Num_corner+j;
        F(3*(id-1)+1:3*id) = Rc_e*M(j,:,i)'+x(4:6)-Puv_c_0(j,:,i)';
        %F(3*(id-1)+1:3*id) = Rc_e*(M(j,:,i)'+x(4:6))-Puv_c_0(j,:,i)';
    end
end


end

