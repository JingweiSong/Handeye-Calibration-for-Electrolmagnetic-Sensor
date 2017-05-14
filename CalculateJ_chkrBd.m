function [ J ] = CalculateJ_chkrBd( Rc_e,x,p_local,chkBoard,num_corners )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function: Calculate Jacobian
%   Method:   See the document
%   Input:    
%             Rc_e:         Rotation from EM sensor to camera
%             x(4:6):       Translation from EM sensor to camera
%             p_local:      Checkerboard pos in checkboard cordinate
%             chkBoard:     Checkerboard pos in global cordinate
%             num_corners:  Number of corners
%   Returns:  
%             J:            Jacobian function
%   Author:   Jingwei Song.   10/05/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
J = zeros(3*num_corners,6);
for i = 1 : num_corners
    J(3*(i-1)+1:3*i,1:3) = -Rc_e*skew(p_local(i,:));
    J(3*(i-1)+1:3*i,4:6) = eye(3,3);
end



end

