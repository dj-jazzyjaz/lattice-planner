width = 600;
height = 600;

map = zeros(height, width);

for i = 1:15
    x0 = randi(width);
    y0 = randi(height);
    sx = randi(60) + 15;
    sy = randi(60) + 15;
    A = randi(30)+1;
    for x_i = 1:width
        for y_i = 1:height
                map(y_i, x_i) = map(y_i, x_i) + gauss_distribution(x_i, y_i, x0, y0, sx, sy, A); 

        end
    end
end

image(map,'CDataMapping','scaled')
colorbar

%% Save to file
fileID = fopen('../maps/map5.txt','w');
    
fprintf(fileID, "N\n%d, %d\nC\n", height, width);
for r=1:height
    for c=1:width-1
        fprintf(fileID, "%d, ", map(r, c));
    end
    fprintf(fileID, "%d\n", map(r, width));
end
fclose(fileID);