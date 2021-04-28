close all;
formatSpec = '%d %d %d %d';
sizePlan = [4 Inf];
file = fopen('plan.txt', 'r');
plan = fscanf(file,formatSpec,sizePlan);
plan = plan';

drone_shape = [[-.025, -.03]; [.025, -.03]; [0, .07]];

Visualize(plan, drone_shape)

function[] = Visualize(plan, drone_shape)

    Intermediate_Pts = load('IntermediatePoints.mat');
    Intermediate_Pts = Intermediate_Pts.Intermediate_Pts;
      
    plan_size = size(plan, 1);
    resolution = .025;
    num_prims_per_angle = 4;
    num_intermediates = 10;
    
    figure();
    hold on;
    axis equal;
    
    for i=1:plan_size
        %draw drone
        th = plan(i, 3) *2*pi/16 -pi/2;
        Xs = drone_shape(:,1)*cos(th) - drone_shape(:,2)*sin(th) + plan(i, 1)*resolution;
        Ys = drone_shape(:,1)*sin(th) + drone_shape(:,2)*cos(th) + plan(i, 2)*resolution;
        plot(polyshape(Xs, Ys), 'FaceColor','red');
        
        %draw primitive
        prim_num = plan(i, 3)*num_prims_per_angle + plan(i, 4);
        Inter_pts = Intermediate_Pts(prim_num*num_intermediates+1:(prim_num+1)*num_intermediates, :);
        plot(Inter_pts(:, 1)+plan(i, 1)*resolution, Inter_pts(:, 2)+plan(i, 2)*resolution, ':', 'linewidth', 2);
        
    end

end
