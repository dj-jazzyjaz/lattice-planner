
function [map] = Make_Map(points)
    width = 10;
    height = 10;
    
    map = zeros(height, width);
    for i=1:size(points, 1)
      point = points(i, :);
      map(point(3), point(2)) = 1;
    end
    
    fileID = fopen('map7.txt','w');
    
    fprintf(fileID, "N\n%d, %d\nC\n", height, width);
    for r=1:height
        for c=1:width-1
            fprintf(fileID, "%d, ", map(r, c));
        end
        fprintf(fileID, "%d\n", map(r, width));
    end
    fclose(fileID);
    
    image(map,'CDataMapping','scaled')
    colorbar
end