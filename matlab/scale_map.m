tablemap = readtable('../maps/map5.txt');
envmap = table2array(tablemap);
map = envmap * 10;
height = 600;
width = 600;

%% Save to file
fileID = fopen('../maps/map8.txt','w');
    
fprintf(fileID, "N\n%d, %d\nC\n", height, width);
for r=1:height
    for c=1:width-1
        fprintf(fileID, "%d, ", map(r, c));
    end
    fprintf(fileID, "%d\n", map(r, width));
end
fclose(fileID);