function plotStates( propagatedState, WP )

    % Calculate corresponding time for each WP
    WP_time = propagatedState.cumulativeTime(WP.WP_index);

    statesToPlot = {'pitchRate' 'headingRate' 'velocity' 'roll' 'arcLength' 'cumulativeTime' 'timeSteps'...
        'pitchWithoutSteps' 'headingWithoutSteps' 'pitch' 'heading' 'dynamicPressure' 'Clalpha'...
        'Cd0' 'L' 'Cl' 'D' 'Cd' 'angleOfAttack' 'lateralGForce' 'north' 'east' 'altitude'};
    for i = 1:numel(statesToPlot)
        state = getfield(propagatedState,statesToPlot{i});
        figure
        hold on
        plot(propagatedState.cumulativeTime(1:numel(state)-2),state(1:numel(state)-2))
        for j = 1:numel(WP_time)
            if j == 1
                xval = 0;
            else
                xval = WP_time(j);
            end
            ymin = min(state(min(WP.WP_index(j),numel(state)-2)),0);
            ymax = max(state(min(WP.WP_index(j),numel(state)-2)),0);
            xPoints = [xval,xval];
            yPoints = [ymin,ymax];
            if WP.WP_types(j) == 1
                plot(xPoints,yPoints,'r--')
                text(xval,ymax,num2str(j-1),'Color','r','VerticalAlignment','bottom')
            else
                plot(xPoints,yPoints,'b--')
                text(xval,ymax,num2str(j-1),'Color','b','VerticalAlignment','bottom')
            end
        end
        hold off
        grid
        margin = 1; % Lateral margins for x-axis in seconds
        xlim([-margin propagatedState.totalTime+margin]);
        title(statesToPlot{i})
        xlabel('Time [s]')
        ylabel(statesToPlot{i})
    end
    
end

