function [ F ] = CalculateF_chkrBd( Rc_e,x,p_local,chkBoard,num_corners )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function: Calculate objective function
%   Method:   See the document
%   Input:    
%             Rc_e:         Rotation from EM sensor to camera
%             x(4:6):       Translation from EM sensor to camera
%             p_local:      Checkerboard pos in checkboard cordinate
%             chkBoard:     Checkerboard pos in global cordinate
%             num_corners:  Number of corners
%   Returns:  
%             F;            Object function
%   Author:   Jingwei Song.   10/05/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
F = zeros(3*num_corners,1);
for i = 1 : num_corners
    F(3*(i-1)+1:3*i) = Rc_e*p_local(i,:)'+x(4:6)-chkBoard(i,:)';
end


end

