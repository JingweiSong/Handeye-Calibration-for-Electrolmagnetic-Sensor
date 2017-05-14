function [ J ] = CalculateJ( Rc_e,x,M,Puv_c_0,Num_frame,Num_corner )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function: Calculate Jacobian
%   Method:   See the document
%   Input:    
%             x(1:3):       Rotation from EM sensor to camera (in smooth
%                           manifold)
%             x(4:6):       Translation from EM sensor to camera
%             M:         	Checkerboard pos in EM sensor cordinate (Num_corner,3,Num_frame)
%             Puv_c_0:      Checkerboard pos in camera cordinate(Num_corner,3,Num_frame)
%             Num_frame:    Number of frames
%             Num_corner:   Number of corner points
%   Returns:  
%             J:            Jacobian function
%   Author:   Jingwei Song.   09/05/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
J = zeros(3*Num_frame*Num_corner,6);
for i = 1 : Num_frame
    for j = 1 : Num_corner
%         J((i-1)*Num_corner+j,1:3) = 2*(Puv_c_0(j,:,i)-M(j,:,i))*Rc_e*skew(M(j,:,i));
%         J((i-1)*Num_corner+j,4:6) = 2*(M(j,:,i)*Rc_e+x(4:6)-Puv_c_0(j,:,i));
        id = (i-1)*Num_corner+j;
        J(3*(id-1)+1:3*id,1:3) = -Rc_e*skew(M(j,:,i));
        J(3*(id-1)+1:3*id,4:6) = eye(3,3);
%         J(3*(id-1)+1:3*id,1:3) = -Rc_e*skew(M(j,:,i)'+x(4:6));
%         J(3*(id-1)+1:3*id,4:6) = Rc_e;
    end
end


end

