function stop = optimplotfunccount(~,optimValues,state,varargin)
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
switch state
    case 'iter'
        if optimValues.iteration == 0
            % The 'iter' case is  called during the zeroth iteration,
            % but it has values that were empty during the 'init' case
            f1 = figure('Visible','Off'); % Create and then hide figure as it is being constructed.
            movegui(f1,'northwest') % Move the GUI to the center of the screen.
            hold on
            WP_ax = scatter3(WP.north,WP.east,WP.up,9,'b','filled');
            traj_ax = plot3(smooth_north,smooth_east,smooth_up);
            hold off
            grid
            title('Trajectory approximation')
            [~,~] = legend('Original points'); % "[~,~]=" prevents the bug in R2015b (https://www.mathworks.com/support/bugreports/1283854)
            axis equal
            axis vis3d % Lock aspect ratio of axes
            view(-45,45); % Azimuth and elevation of initial view (degrees)
            xlabel('North')
            ylabel('East')
            zlabel('Up')
            f1.Visible = 'on';
        else
            % Not the zeroth iteration
            WP_ax.XData = WP.north;
            WP_ax.YData = WP.east;
            WP_ax.ZData = WP.up;
            traj_ax.XData = smooth_north;
            traj_ax.YData = smooth_east;
            traj_ax.ZData = smooth_up;
        end
end
