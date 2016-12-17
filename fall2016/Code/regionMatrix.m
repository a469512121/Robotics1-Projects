function M = regionMatrix(image,y,x)
% input: x and y are pixal coordinate in image, 
% output: 51x51 matrix with a center at x,y
M = zeros(51,51);
image(y,x);
for col=-25:25
    for row=-25:25
        M(col+26,row+26) = image(col+y,row+x);     
    end
end
end 
