
%MOUSE_POINTS      Input nodes from a grid.
%   Mouse_Points allows to input points using a variation of ginput 
%   Matlab function. The points will snap to the grid. The function returns
%   the [n X Y] matrix (n = number of non repeated nodes, X = x coordinate,
%   Y = y coordinate). The function is mouse friendly.
%
%   Left Click                 : add a point
%   Scroll                     : zoom in/out
%   Scroll Wheel click         : pan
%   Double Scroll Wheel  click : reset view to default view
%   Right click                : set new default view
%   Enter                      : return [n X Y] matrix
%
%   [points]=Mouse_Points(GridLimits) returns [n X Y] matrix input data 
%   from a Grid with initial limits [-GridLimits Gridlimits] on x axis and
%   [-GridLimits Gridlimits] on y axis.
%
%   [points]=Mouse_Points returns [n X Y] matrix input data from a Grid
%   with initial limits [-1 1] on x axis and [-1 1] on y axis.
%   
% EXAMPLE:
%
% [PointsMatrix]=Mouse_Points(100);
%   
% See also ginput, figure, zoom, pan.
%
% This function is still in develop, so please report any bugs, suggestions  
% or inquiries to: 
%
% Franklin F. Lucero.
% Contact:
%           franklin.lucero94@gmail.com    (personal)
%           franklin.lucero@ucuenca.ec     (university)
% Institution: Universidad de Cuenca
%
% Special thanks to Rody P.S. Oldenhuis, developer of mouse_figure
% function,that allows a mouse friendly handle of figures.
%
% In case you liked this work, please consider a donation to help me finish my studies: 
% https://www.paypal.com/cgi-bin/webscr?cmd=_donations&business=2EW5AXXJ7TGH8&lc=US&item_name=Franklin%20F%2e%20Lucero&no_note=0&cn=Dar%20instrucciones%20especiales%20al%20vendedor%3a&no_shipping=2&currency_code=USD&bn=PP%2dDonationsBF%3abtn_donateCC_LG%2egif%3aNonHosted
% 
%**************************************************************************
%**************************************************************************
% Function Begins
function [nodes]=Mouse_Points(GridLimits)
nodes=[];
button=inf;
% Checks input to define grid initial limits
if      nargin  == 1
    limits  =  [-GridLimits GridLimits];
elseif  nargin  == 0
    limits  =  [-1 1];
end
% Draws initial grid maximizing its size
sz=get(0,'ScreenSize');              
fig=figure('position',[sz(1) sz(2) sz(3) sz(4)]);
title('Click to add nodes, ENTER to finish','FontSize',16)
xlabel('X','FontWeight','bold','FontSize',14)
ylabel('Y','FontWeight','bold','FontSize',14)
xlim(limits);
ylim(limits);
grid on
set(gca,'Xtick',0:1:GridLimits)
set(gca,'Ytick',0:1:GridLimits)
mouse_handle(fig);
hold on
% Initialize points data input 
while button ~= 13      
    
    % waits for a button to be presses
    keydown = waitforbuttonpress;
    
    % Identifies the button pressed
    if keydown
        button = abs(get(fig, 'CurrentCharacter'));
    else
        button = get(fig, 'SelectionType');
        if strcmp(button,'open')
            button = 3;
        elseif strcmp(button,'normal')
            button = 1;
        elseif strcmp(button,'extend')
            button = 2;
        elseif strcmp(button,'alt')
            button = 3;
        else
            error(message('Invalid Button Selection'))
        end
    end
    
    % Obtains the current point
    pt = get(gca, 'CurrentPoint');
    % Get grid points
    xongrid=get(gca, 'XTick');
    yongrid=get(gca, 'YTick');
    % Snap to grid 
    [~,positiongrid]=min(abs(pt(1,1)-xongrid));
    closerxgrid=xongrid(positiongrid);
    pt(1,1)=closerxgrid;
    [~,positiongrid]=min(abs(pt(1,2)-yongrid));
    closerygrid=yongrid(positiongrid);
    pt(1,2)=closerygrid;
    % Plots points when mouse left button is pressed
    if button==1 
        plot(pt(1,1),pt(1,2),'s','Color','blue')
        %text(pt(1,1),0.95*pt(1,2),[num2str(pt(1,1)) ' ; ' num2str(pt(1,2))],'Color',[0.3 0.3 0.3],'FontSize',8)
        nodes = [nodes; 1 pt(1,1) pt(1,2)]; %#ok<AGROW>
        
        % Deletes repeated nodes
        if length(nodes)>=2
            if isempty(find(nodes(1:end-1,2)==pt(1,1), 1))==0   % Checks if x exists
                xcoincidences=find(nodes(1:end-1,2)==pt(1,1));
                for k = 1: length(xcoincidences)                % Checks if point existed before
                    if nodes(xcoincidences(k),2)==pt(1,1) && nodes(xcoincidences(k),3)==pt(1,2)
                        nodes=nodes(1:end-1,:);                 % Don't add to the node matrix
                    end
                end
            end
            
            % adds number of node
            nodes(:,1)=1:size(nodes,1);
        end
    end
end
% Identify nodes
for k=1:size(nodes,1)
    text(nodes(k,2),nodes(k,3),num2str(k),'FontSize',16,'FontWeight','bold')
end
    
end
function MainFig = mouse_handle(MainFig)
    % initialize
    status = '';  previous_point = [];
               
    % get original limits
    axs = get(MainFig, 'currentaxes');
    original_xlim = get(axs, 'xlim');
    original_ylim = get(axs, 'ylim');
    
    % define zooming with scrollwheel, and panning with scrollclicks
    set(MainFig, ...
        'WindowScrollWheelFcn' , @scroll_zoom,...
        'WindowButtonDownFcn'  , @pan_click,...
        'WindowButtonUpFcn'    , @pan_release,...
        'WindowButtonMotionFcn', @pan_motion);    
    % zoom in to the current point with the mouse wheel
    function scroll_zoom(varargin)
        % double check if these axes are indeed the current axes
        if get(MainFig, 'currentaxes') ~= axs, return, end
        % get the amount of scolls
        scrolls = varargin{2}.VerticalScrollCount;
        % get the axes' x- and y-limits
        xlim = get(axs, 'xlim');  ylim = get(axs, 'ylim');
        % get the current camera position, and save the [z]-value
        cam_pos_Z = get(axs, 'cameraposition');  cam_pos_Z = cam_pos_Z(3);
        % get the current point
        old_position = get(axs, 'CurrentPoint'); old_position(1,3) = cam_pos_Z;
        % calculate zoom factor
        zoomfactor = 1 - scrolls/50;
        % adjust camera position
        set(axs, 'cameratarget', [old_position(1, 1:2), 0],...
            'cameraposition', old_position(1, 1:3));
        % adjust the camera view angle (equal to zooming in)
        camzoom(zoomfactor);
        % zooming with the camera has the side-effect of
        % NOT adjusting the axes limits. We have to correct for this:
        x_lim1 = (old_position(1,1) - min(xlim))/zoomfactor;
        x_lim2 = (max(xlim) - old_position(1,1))/zoomfactor;
        xlim   = [old_position(1,1) - x_lim1, old_position(1,1) + x_lim2];
        y_lim1 = (old_position(1,2) - min(ylim))/zoomfactor;
        y_lim2 = (max(ylim) - old_position(1,2))/zoomfactor;
        ylim   = [old_position(1,2) - y_lim1, old_position(1,2) + y_lim2];
        set(axs, 'xlim', xlim), set(axs, 'ylim', ylim)
        % set new camera position
        new_position = get(axs, 'CurrentPoint');
        old_camera_target =  get(axs, 'CameraTarget');
        old_camera_target(3) = cam_pos_Z;
        new_camera_position = old_camera_target - ...
            (new_position(1,1:3) - old_camera_target(1,1:3));
        % adjust camera target and position
        set(axs, 'cameraposition', new_camera_position(1, 1:3),...
            'cameratarget', [new_camera_position(1, 1:2), 0]);
        % we also have to re-set the axes to stretch-to-fill mode
        set(axs, 'cameraviewanglemode', 'auto',...
            'camerapositionmode', 'auto',...
            'cameratargetmode', 'auto');
    end % scroll_zoom
    % pan upon scroll click
    function pan_click(varargin)
        % double check if these axes are indeed the current axes
        if get(MainFig, 'currentaxes') ~= axs, return, end
        % perform appropriate action
        switch lower(get(MainFig, 'selectiontype'))            
            % start panning on left click
            case 'extend' 
                status = 'down';
                previous_point = get(axs, 'CurrentPoint');              
            % reset view on double click
            case 'open' % double click (left or right)
                set(axs, 'Xlim', original_xlim,...
                         'Ylim', original_ylim);  
            % right click - set new reset state
            case 'alt'
                original_xlim = get(axs, 'xlim');
                original_ylim = get(axs, 'ylim');
        end
    end
    % release scroll button
    function pan_release(varargin)
        % double check if these axes are indeed the current axes
        if get(MainFig, 'currentaxes') ~= axs, return, end
        % just reset status
        status = '';
    end
    % move the mouse (with scroll button clicked)
    function pan_motion(varargin)
        % double check if these axes are indeed the current axes
        if get(MainFig, 'currentaxes') ~= axs, return, end
        % return if there isn't a previous point
        if isempty(previous_point), return, end  
        % return if mouse hasn't been clicked
        if isempty(status), return, end  
        % get current location (in pixels)
        current_point = get(axs, 'CurrentPoint');
        % get current XY-limits
        xlim = get(axs, 'xlim');  ylim = get(axs, 'ylim');     
        % find change in position
        delta_points = current_point - previous_point;  
        % adjust limits
        new_xlim = xlim - delta_points(1); 
        new_ylim = ylim - delta_points(3); 
        
        % set new limits
        set(axs, 'Xlim', new_xlim); set(axs, 'Ylim', new_ylim);           
        % save new position
        previous_point = get(axs, 'CurrentPoint');
    end 
    
end
