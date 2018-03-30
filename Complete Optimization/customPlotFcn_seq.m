function stop = customPlotFcn_seq(IP_seq,optimValues,state,~,IP,WP,index)
% OPTIMPLOTFUNCCOUNT Plot number of function evaluations at each iteration.
%
%   STOP = OPTIMPLOTFUNCCOUNT(X,OPTIMVALUES,STATE) plots the value in
%   OPTIMVALUES.funccount.
%
%   Example:
%   Create an options structure that will use OPTIMPLOTFUNCCOUNT as the
%   plot function
%     options = optimset('PlotFcns',@optimplotfunccount);
%
%   Pass the options into an optimization solver to view the plot
%     fminbnd(@sin,3,10,options)

%   Copyright 2006-2010 The MathWorks, Inc.

stop = false;
WP_ax = findobj(get(gca,'Children'),'Tag','WP_ax_tag');
traj_ax = findobj(get(gca,'Children'),'Tag','traj_ax_tag');
WP_all_ax = findobj(get(gca,'Children'),'Tag','WP_all_ax_tag');
switch state
    case 'init'
        % clean up plot from a previous run, if any
        if ~isempty(WP_ax)
            delete(WP_ax);
        end
        if ~isempty(traj_ax)
            delete(traj_ax);
        end
        if ~isempty(WP_all_ax)
            delete(WP_all_ax);
        end
    case 'iter'
        % Obtain new way-point sequence
        [numOfWaypoints,~] = size(WP.north);
        j = 0;
        k = 0;
        new_size = (2*numOfWaypoints)-1;
        new_north = zeros(1,new_size);
        new_east = zeros(1,new_size);
        for i = 1:new_size
            if i == 2*index
                k = k + 1;
                new_north(i) = IP_seq(1);
                new_east(i) = IP_seq(2);
            elseif mod(i,2) == 1
                j = j + 1;
                new_north(i) = WP.north(j);
                new_east(i) = WP.east(j);
            else
                k = k + 1;
                new_north(i) = IP(k);
                new_east(i) = IP(k+numOfWaypoints-1);
            end
        end
        new_up = 20*ones(1,new_size);
        % Spline generation along way-point sequence
        N = 250; % Number of uniformly distributed points along the curve parameter
        smoothTraj = cscvn([new_north;new_east;new_up]);
        space = linspace(smoothTraj.breaks(1),smoothTraj.breaks(end),N);
        smooth = fnval(smoothTraj,space);
        smooth_north = smooth(1,:)';
        smooth_east = smooth(2,:)';
        smooth_up = smooth(3,:)';
        if optimValues.iteration == 0
            % The 'iter' case is  called during the zeroth iteration,
            % but it has values that were empty during the 'init' case
            hold on
            WP_all_ax = scatter3(new_north,new_east,new_up,9,'r','filled');
            WP_fixed_ax = scatter3(WP.north,WP.east,WP.up,9,'b','filled');
            traj_ax = plot3(smooth_north,smooth_east,smooth_up);
            set(WP_fixed_ax,'Tag','WP_ax_tag');
            set(traj_ax,'Tag','traj_ax_tag');
            set(WP_all_ax,'Tag','WP_all_ax_tag');
            hold off
            grid
            title(['Total time ' num2str(optimValues.fval)...
                's (' num2str(optimValues.funccount) ' function evaluations)'])
            axis equal
            axis vis3d % Lock aspect ratio of axes
            view(-45,30); % Azimuth and elevation of initial view (degrees)
            xlabel('North')
            ylabel('East')
            %zlabel('Up')
        else
            % Not the zeroth iteration
            set(WP_all_ax,'XData',new_north,'YData',new_east,'ZData',new_up);
            set(traj_ax,'XData',smooth_north,'YData',smooth_east,'ZData',smooth_up);
            title(['Total time ' num2str(optimValues.fval)...
                's (' num2str(optimValues.funccount) ' function evaluations)'])
        end
end
