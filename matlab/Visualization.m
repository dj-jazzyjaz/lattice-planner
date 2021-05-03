close all;
formatSpec = '%d %d %d %d';
sizePlan = [4 Inf];
file = fopen('plan.txt', 'r');
plan = fscanf(file,formatSpec,sizePlan);
plan = plan';

drone_shape = [[-.25, -.3]; [.25, -.3]; [0, .7]];

tablemap = readtable('../maps/map2.txt');
envmap = table2array(tablemap);


Visualize(plan, drone_shape, envmap)

function[] = Visualize(plan, drone_shape, map)

    c_map = [ 1 1 1 ; 1 1 .5 ; 1 .75 0 ; 1 .5 0 ; 1 0 0];
    figure(); % 'units','normalized','outerposition',[0 0 1 1]);
    imagesc(map); axis square; colorbar; colormap(c_map); hold on;
    drone_scale = 4;

    Intermediate_Pts = load('IntermediatePoints.mat');
    Intermediate_Pts = Intermediate_Pts.Intermediate_Pts;
      
    plan_size = size(plan, 1);
    resolution = 1; %.025;
    scale = 40; % 
    num_prims_per_angle = 4;
    num_intermediates = 10;
    
%     figure()
%     hold on;
%     axis equal;
     
  
    
    for i=1:plan_size
       
        switch plan(i, 4)
            case 0
                color = [.7 0 1];
            case 1
                color = [0 1 0];
            case 2
                color = [0 0 1];
            case 3
                color = [1 .5 0];
        end
       
        % draw map
%         for i=1:size(map, 1)
%             for j=1:size(map, 2)
%                 x0 = 1+i*resolution;
%                 y0 = 1+j*resolution;
%                 x1 = x0 + resolution;
%                 y1 = y0 + resolution;
%                 map_c = 1-map(i, j)/10;
%                 plot(polyshape([x0, x0, x1, x1],[y0, y1, y1, y0]), 'facecolor', [map_c, map_c, map_c]);
%             end
%         end

        %draw drone
        th = mod(plan(i, 3), 16) *2*pi/16 -pi/2;
   
        Xs = drone_shape(:,1)*cos(th)*drone_scale - drone_shape(:,2)*sin(th)*drone_scale + plan(i, 1)*resolution;
        Ys = drone_shape(:,1)*sin(th)*drone_scale + drone_shape(:,2)*cos(th)*drone_scale + plan(i, 2)*resolution;
        plot(polyshape(Xs, Ys), 'FaceColor','red');
        
        %draw primitive
        prim_num = mod(plan(i, 3),16)*num_prims_per_angle + plan(i, 4);
        Inter_pts = Intermediate_Pts(prim_num*num_intermediates+1:(prim_num+1)*num_intermediates, :);
        plot(Inter_pts(:, 1)*scale+plan(i, 1)*resolution, Inter_pts(:, 2)*scale+plan(i, 2)*resolution, ':', 'Color', color, 'linewidth', 2);
        
    end

end
