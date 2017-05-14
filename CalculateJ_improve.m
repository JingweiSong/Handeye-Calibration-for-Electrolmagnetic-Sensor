function [ J ] = CalculateJ( Rc_e,x,Re_g,Te_g,Puv_c_0,Num_frame,Num_corner,chkBoard )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function: Calculate Jacobian
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
%             J:            Jacobian function
%   Author:   Jingwei Song.   13/05/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Numerical estimation
delta  = 0.00000000001;
deltax = [delta 0   0   ]';
deltay = [0   delta 0   ]';
deltaz = [ 0    0  delta]';

J = zeros(3*Num_frame*Num_corner+6*Num_frame,6*Num_frame+6);

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
        d = x(6*(i-1)+1:6*(i-1)+3);
        %J(3*(id-1)+1:3*id,6*(i-1)+1:6*(i-1)+3) = Rc_e*Re_g(:,:,i)'* (-expm(skew(d))) * skew(chkBoard(j,:));
        %   Numerical 
        J(3*(id-1)+1:3*id,6*(i-1)+1) = ( Rc_e*Re_g(:,:,i)'*expm(skew(d+deltax))*chkBoard(j,:)' - Rc_e*Re_g(:,:,i)'*expm(skew(d))*chkBoard(j,:)' ) / delta;
        J(3*(id-1)+1:3*id,6*(i-1)+2) = ( Rc_e*Re_g(:,:,i)'*expm(skew(d+deltay))*chkBoard(j,:)' - Rc_e*Re_g(:,:,i)'*expm(skew(d))*chkBoard(j,:)' ) / delta;
        J(3*(id-1)+1:3*id,6*(i-1)+3) = ( Rc_e*Re_g(:,:,i)'*expm(skew(d+deltaz))*chkBoard(j,:)' - Rc_e*Re_g(:,:,i)'*expm(skew(d))*chkBoard(j,:)' ) / delta;
        
        J(3*(id-1)+1:3*id,6*(i-1)+4:6*(i-1)+6) = Rc_e;
        J(3*(id-1)+1:3*id,end-5:end-3) =  -Rc_e*skew(M(j,:,i));
        J(3*(id-1)+1:3*id,end-2:end  ) =  eye(3,3);
    end
end

%   PartII & III: xi ti
start = 3*Num_frame*Num_corner;
for i = 1 : Num_frame
     J(start+6*(i-1)+1:start+6*(i-1)+6,6*(i-1)+1:6*(i-1)+6) = eye(6,6);
end

end

