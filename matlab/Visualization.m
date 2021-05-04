close all;
formatSpec = '%d %d %d %d %d';
sizePlan = [5 Inf];
file = fopen('plan1.txt', 'r');
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

    Intermediate_Pts_Highres = load('IntermediatePointsHighRes.mat');
    Intermediate_Pts_Highres = Intermediate_Pts_Highres.Intermediate_Pts_Highres;
    
    Intermediate_Pts_Lowres = load('IntermediatePointsLowRes.mat');
    Intermediate_Pts_Lowres = Intermediate_Pts_Lowres.Intermediate_Pts_Lowres;
      
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
            th = mod(plan(i, 3), 8) *2*pi/8 -pi/2;
        else
            th = mod(plan(i, 3), 16) *2*pi/16 -pi/2;
        end
   
        Xs = drone_shape(:,1)*cos(th)*drone_scale - drone_shape(:,2)*sin(th)*drone_scale + plan(i, 1)*resolution;
        Ys = drone_shape(:,1)*sin(th)*drone_scale + drone_shape(:,2)*cos(th)*drone_scale + plan(i, 2)*resolution;
        plot(polyshape(Xs, Ys), 'FaceColor','red');
        
        %draw primitive
        if(i ~= size(plan, 1)) % no primitive for last point
            if (low_res)
                prim_num = mod(plan(i, 3), 8)*num_prims_per_angle + plan(i, 4);
            else
                prim_num = mod(plan(i, 3),16)*num_prims_per_angle + plan(i, 4);
            end
            if (~low_res)
                Inter_pts = Intermediate_Pts_Highres(prim_num*num_intermediates+1:(prim_num+1)*num_intermediates, :);
            elseif (low_res)
                Inter_pts = Intermediate_Pts_Lowres(prim_num*num_intermediates+1:(prim_num+1)*num_intermediates, :);
            end
            plot(Inter_pts(:, 1)*scale+plan(i, 1)*resolution, Inter_pts(:, 2)*scale+plan(i, 2)*resolution, ':', 'Color', color, 'linewidth', 2);
        end
    end

end
