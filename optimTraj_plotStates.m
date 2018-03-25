function optimTraj_plotStates( propagatedState, WP_location )

    % Calculate corresponding time for each WP
    WP_time = propagatedState.cumulativeTime(WP_location);

    statesToPlot = {'headingRate' 'velocity' 'roll'};
    for i = 1:numel(statesToPlot)
        state = getfield(propagatedState,statesToPlot{i});
        figure
        hold on
        plot(propagatedState.cumulativeTime(1:numel(state)-2),state(1:numel(state)-2))
        for j = 1:numel(WP_time)
            xval = WP_time(j);
            ymin = min(state(min(WP_location(j),numel(state)-2)),0);
            ymax = max(state(min(WP_location(j),numel(state)-2)),0);
            xPoints = [xval,xval];
            yPoints = [ymin,ymax];
            if mod(j,2) == 0
                plot(xPoints,yPoints,'r--')
                text(xval,ymax,num2str(j-1),'Color','r','VerticalAlignment','bottom')
            else
                plot(xPoints,yPoints,'b--')
                text(xval,ymax,num2str(j-1),'Color','b','VerticalAlignment','bottom')
            end
        end
        hold off
        grid
        title(statesToPlot{i})
        xlabel('Time [s]')
        ylabel(statesToPlot{i})
    end
    
end

