width = 600;
height = 600;

map = ones(height, width);

for i = 1:120
    x0 = randi(width);
    y0 = randi(height);
    sx = randi(60) + 25;
    sy = randi(60) + 25;
    A = randi(5)+1;
    for dx = 1:sx
        for dy = 1:sy
                newy = y0 + dy;
                newx = x0 + dx;
                if(newy <= height && newx <= width)
                    map(y0 + dy, x0 + dx) = A; 
                end

        end
    end
end

for x = [200:220 400:420]
    for y = 1:height
        map(y, x) = 0;
    end
end

for y = [200:220 400:420]
    for x = 1:width
        map(y, x ) = 0;
    end
end

map = round(map)
image(map,'CDataMapping','scaled')
colorbar

%% Save to file
fileID = fopen('../maps/map7.txt','w');
    
fprintf(fileID, "N\n%d, %d\nC\n", height, width);
for r=1:height
    for c=1:width-1
        fprintf(fileID, "%d, ", map(r, c));
    end
    fprintf(fileID, "%d\n", map(r, width));
end
fclose(fileID);