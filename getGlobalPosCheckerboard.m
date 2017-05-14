function [ chkBoard_opt,error ] = getGlobalPosCheckerboard( chkBoard,num_row,num_col,squareSize )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function: Calculate Jacobian
%   Method:   Variables: Rotation and translation from local corners to 
%                        EM sensor global position
%                        First construct the checker board corners in local 
%                        And find optimal solution from transformed
%                        postion to global measurements
%   Input:    
%             chkBoard:     corners measured by EM. (3*Num_corner)
%             num_row:   Number of rows 
%             num_col:   Number of columns 
%             squareSize:   Square size of the checkerboard
%   Returns:  
%             chkBoard:     Optimized chkBoard
%             error:     Distance between optimized corners to measurements
%   Author:   Jingwei Song. 10/05/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Construct corners' postion in local cordinate
p_local = zeros(num_row*num_col,3);     
for i = 1 : num_col
    for j = 1 : num_row
        p_local((i-1)*num_row+j,:) = [(i-1)*squareSize,(j-1)*squareSize,0];
    end
end

%   Initialize variable  [rotation translation]
x = zeros(6,1); 
Rc_e = eye(3,3);

F = CalculateF_chkrBd( Rc_e,x,p_local,chkBoard,num_row*num_col );
J = CalculateJ_chkrBd( Rc_e,x,p_local,chkBoard,num_row*num_col );
P = diag(ones(size(F,1),1));
min_FX_old = 10000000000000;
k=0;
I = eye(size(J,2));
M = 0.00000000000001;
while ((F'*P*F)>M&&abs(F'*P*F-min_FX_old)>M&&k<100)
    min_FX_old = F'*P*F
    J = sparse(J);  
    F = sparse(F);
    
    u = 0.01;
    d = -(J'*P*J+u*I)\(J'*P*F);
    x_new = x + d;
    Rc_e_new = Rc_e*expm(skew(d(1:3)));
    F_new = CalculateF_chkrBd( Rc_e_new,x_new,p_local,chkBoard,num_row*num_col );
    min_FX = F_new'*P*F_new;
    t = 1;
    while(min_FX > min_FX_old&&t<10)
        % LM algorithm
        d = -(J'*P*J+u*I)\(J'*P*F);
        x_new = x + d;
        Rc_e_new = Rc_e*expm(skew(d(1:3)));
        F_new = CalculateF_chkrBd( Rc_e_new,x_new,p_local,chkBoard,num_row*num_col );
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
    F = CalculateF_chkrBd( Rc_e,x,p_local,chkBoard,num_row*num_col );
    J = CalculateJ_chkrBd( Rc_e,x,p_local,chkBoard,num_row*num_col );
end

%   Calculate optimized chkBoard
chkBoard_opt = zeros(size(chkBoard));
for i = 1 : num_row*num_col
    chkBoard_opt(i,:) = Rc_e*p_local(i,:)'+x(4:6);;
end

%   Estimate error
error = 0;
for i = 1 : num_row*num_col
    tmp = chkBoard_opt(i,:) - chkBoard(i,:);
    error = error + sqrt(tmp * tmp');
end
error = error / (num_row*num_col);

end

