close all;
formatSpec = '%d %d %d %d %d %d';
sizePlan = [6 Inf];
file = fopen('plan_around_the_mountain.txt', 'r');
text = textscan(file,formatSpec, 'headerlines', 8);

cols = size(text, 2);
rows = size(cell2mat(text(1)), 1);
plan = zeros(rows, cols);
for i =1:cols
    plan(:, i) = (cell2mat(text(i)));
end

drone_shape = [[0 2 0 0]; [0 .5 1 .5]; [0 .5 0 1]]; % x, y, z
% highres_drone = stlread('drone1.stl');

tablemap = readtable('../maps/map5.txt');
envmap = table2array(tablemap);


Visualize(plan, drone_shape, envmap)

function[] = Visualize(plan, drone_shape, map)
    z_scale = 10;
    c_map = [ 1 1 1 ; 1 1 .5 ; 1 .75 0 ; 1 .5 0 ; 1 0 0];
    figure(); % 'units','normalized','outerposition',[0 0 1 1]);
    %imagesc(map); axis square; colorbar; colormap(c_map); hold on;
    hold on;
    [X, Y] = meshgrid(1:size(map, 1), 1:size(map, 2));
    s = surf(X, Y, z_scale * map);
    s.EdgeColor = 'none';
    colorbar;
    drone_scale = 5;

    Intermediate_Pts_Highres = load('IntermediatePointsHighRes3D.mat');
    Intermediate_Pts_Highres = Intermediate_Pts_Highres.Intermediate_Pts_Highres3D;
    
    Intermediate_Pts_Lowres = load('IntermediatePointsLowRes3D.mat');
    Intermediate_Pts_Lowres = Intermediate_Pts_Lowres.Intermediate_Pts_Lowres3D;
      
    plan_size = size(plan, 1);
    resolution = 1; %.025;
    scale = 40; % 
    num_prims_per_angle = 4;
    num_intermediates = 10;
    
%     figure()
%     hold on;
%     axis equal;
     
  
    
    for i=1:plan_size
        
        low_res = plan(i, 5);
       
%         switch plan(i, 4)
%             case 0
%                 color = [.7 0 1];
%             case 1
%                 color = [0 1 0];
%             case 2
%                 color = [0 0 1];
%             case 3
%                 color = [1 .5 0];
%         end

        if (low_res)
            color = [.7 0 1];
        else
            color = [.2 1 .7];
        end
            
       

        %draw drone
        if (low_res)
            th = mod(plan(i, 3), 8) *2*pi/8; % -pi/2;
        else
            th = mod(plan(i, 3), 16) *2*pi/16; % -pi/2;
        end
   
        Xs = drone_shape(1,:)*cos(th)*drone_scale - drone_shape(2,:)*sin(th)*drone_scale + plan(i, 1)*resolution;
        Ys = drone_shape(1,:)*sin(th)*drone_scale + drone_shape(2,:)*cos(th)*drone_scale + plan(i, 2)*resolution;
        Zs = drone_shape(3,:)*drone_scale/z_scale + plan(i, 6)*resolution;
        Zs = z_scale * Zs;
        shp = alphaShape(Xs', Ys', Zs');
        shape = plot(shp);
        shape.FaceColor = 'r';
        shape.FaceAlpha = .25;
        text(Xs(1), Ys(1), Zs(1), int2str(i), 'fontsize', 10);
        
        %draw primitive
        if(i ~= 1) % no primitive for last point
            prim_num = plan(i, 4); 
            if (~low_res)
                Inter_pts = Intermediate_Pts_Highres(prim_num*num_intermediates+1:(prim_num+1)*num_intermediates, :);
            elseif (low_res)
                Inter_pts = Intermediate_Pts_Lowres(prim_num*num_intermediates+1:(prim_num+1)*num_intermediates, :);
            end
            plot3(Inter_pts(:, 1)*scale+plan(i-1, 1)*resolution, Inter_pts(:, 2)*scale+plan(i-1, 2)*resolution, (Inter_pts(:, 4)*scale+plan(i-1, 6)*resolution) * z_scale, ':', 'Color', color, 'linewidth', 2);
        end
    end

end
