function pt = point_searching(Ia,Ib,F,x_1,y_1)
% input: two images, fundamental matrix and searching point from image 1
% output: x, y in image 2 with pixal coord

pt_region_1 = regionMatrix(Ia,y_1,x_1);
ep_line = F*[x_1;y_1;1];
list_1 = [];
if (abs(ep_line(2))<=0.001)
    for j=26:1:size(Ib,1)-26
    i = (-1/ep_line(1))*(ep_line(2)*j+ep_line(3));
    if (26<i && i<size(Ib,1)-26)
        pt_region_2 = regionMatrix(Ib,int16(j),int16(i));
        error = [sum(sum(abs(pt_region_2-pt_region_1))) i j];
        list_1 = [list_1;error];
    end
end
    
else
    for i=26:1:size(Ib,2)-26
    j = (-1/ep_line(2))*(ep_line(1)*i+ep_line(3));
        if (26<j && j<size(Ib,1)-26)
            pt_region_2 = regionMatrix(Ib,int16(j),i);
            error = [sum(sum(abs(pt_region_2-pt_region_1))) i j];
            list_1 = [list_1;error];
        end
    end
end
index = find(list_1(:,1) == min(list_1(:,1)));
pt = list_1(index(1),2:3);