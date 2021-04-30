
function [map] = Make_Map(points)
    width = 100;
    height = 100;
    
    map = zeros(height, width);
    for i=1:size(points, 1)
       point = points(i, :);
       map(point(3), point(2)) = 1;
    end
    
    fileID = fopen('map6.txt','w');
    
    for r=1:height
        for c=1:width-1
            fprintf(fileID, "%d, ", map(r, c));
        end
        fprintf(fileID, "%d\n", map(r, width));
    end
    fclose(fileID);
end