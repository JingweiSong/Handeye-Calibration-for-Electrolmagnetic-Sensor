p = 200;   %    Pixel size
m = 5;     %    image size (row)
n = 4;     %    image size (col)

I = checkerboard(p,m,n);
K = I > 0.5;



%   Eliminate one COL on the right
K = K(:,1:size(K,2)* (2*n-1)/(2*n));


imshow(K);
imwrite(K,'checkerboard.tif');