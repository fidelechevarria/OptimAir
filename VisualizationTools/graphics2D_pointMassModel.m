function graphics2D_pointMassModel(WP,totalTrajectory,configuration)

    % Select variables to plot
    plotStates = configuration.plotStates; % Without quaternions
    plotControls = configuration.plotControls;
    plotEuler = configuration.plotStates;
    plotQuatNormError = configuration.plotStates;
    plotAccels = configuration.plotStates;

    % Calculate corresponding time for each WP
    middlePositions = [];
    for i = 1:totalTrajectory.numOfSegments-1
        middlePositions = [middlePositions totalTrajectory.segmentSize*i];
    end
    WP_time = [0 totalTrajectory.time(middlePositions) totalTrajectory.totalTime];
    WP_index = [1 middlePositions totalTrajectory.numOfPoints];

    % PLOT STATES
    if plotStates
        statesToPlot = {'Velocity [m/s]' 'q0' 'q1' 'q2' 'q3'...
            'North Position [m]' 'East Position [m]' 'Altitude [m]'};
        for i = 1:totalTrajectory.numOfStates
            if (i > 1) && (i < 6)
                % Do not plot quaternions
            else
                figure
                hold on
                plot(totalTrajectory.time,totalTrajectory.states(i,:))
                for j = 1:numel(WP_time)
                    xval = WP_time(j);
                    ymin = min(totalTrajectory.states(i,WP_index(j)),0);
                    ymax = max(totalTrajectory.states(i,WP_index(j)),0);
                    xPoints = [xval,xval];
                    yPoints = [ymin,ymax];
                    plot(xPoints,yPoints,'r--')
                    text(xval,ymax,num2str(j),'Color','r','VerticalAlignment','bottom')
                end
                hold off
                grid
                margin = totalTrajectory.totalTime/20; % Lateral margins for x-axis in seconds
                xlim([-margin totalTrajectory.totalTime+margin]);
                title(statesToPlot{i})
                xlabel('Time [s]')
                ylabel(statesToPlot{i})
            end
        end
    end
        
    % PLOT CONTROLS
    if plotControls
        controlsToPlot = {'Angle of Attack [º]' 'Thrust [N]'...
            'Angular Velocity in x-axis [º/s]'};
        for i = 1:totalTrajectory.numOfControls
            if i == 1 || i == 3 % Convert from radians to degrees
                figure
                hold on
                plot(totalTrajectory.time,rad2deg(totalTrajectory.controls(i,:)))
                for j = 1:numel(WP_time)
                    xval = WP_time(j);
                    ymin = min(rad2deg(totalTrajectory.controls(i,WP_index(j))),0);
                    ymax = max(rad2deg(totalTrajectory.controls(i,WP_index(j))),0);
                    xPoints = [xval,xval];
                    yPoints = [ymin,ymax];
                    plot(xPoints,yPoints,'r--')
                    text(xval,ymax,num2str(j),'Color','r','VerticalAlignment','bottom')
                end
                hold off
                grid
                margin = totalTrajectory.totalTime/20; % Lateral margins for x-axis in seconds
                xlim([-margin totalTrajectory.totalTime+margin]);
                title(controlsToPlot{i})
                xlabel('Time [s]')
                ylabel(controlsToPlot{i})
            else
                figure
                hold on
                plot(totalTrajectory.time,totalTrajectory.controls(i,:))
                for j = 1:numel(WP_time)
                    xval = WP_time(j);
                    ymin = min(totalTrajectory.controls(i,WP_index(j)),0);
                    ymax = max(totalTrajectory.controls(i,WP_index(j)),0);
                    xPoints = [xval,xval];
                    yPoints = [ymin,ymax];
                    plot(xPoints,yPoints,'r--')
                    text(xval,ymax,num2str(j),'Color','r','VerticalAlignment','bottom')
                end
                hold off
                grid
                margin = totalTrajectory.totalTime/20; % Lateral margins for x-axis in seconds
                xlim([-margin totalTrajectory.totalTime+margin]);
                title(controlsToPlot{i})
                xlabel('Time [s]')
                ylabel(controlsToPlot{i})
            end
        end
    end
    
    % PLOT EULER ANGLES (Convert from radians to degrees)
    if plotEuler
        eulerToPlot = {'Yaw [º]' 'Pitch [º]' 'Roll [º]'};
        for i = 1:3
            figure
            hold on
            plot(totalTrajectory.time,rad2deg(totalTrajectory.euler(i,:)))
            for j = 1:numel(WP_time)
                xval = WP_time(j);
                ymin = min(rad2deg(totalTrajectory.euler(i,WP_index(j))),0);
                ymax = max(rad2deg(totalTrajectory.euler(i,WP_index(j))),0);
                xPoints = [xval,xval];
                yPoints = [ymin,ymax];
                plot(xPoints,yPoints,'r--')
                text(xval,ymax,num2str(j),'Color','r','VerticalAlignment','bottom')
            end
            hold off
            grid
            margin = totalTrajectory.totalTime/20; % Lateral margins for x-axis in seconds
            xlim([-margin totalTrajectory.totalTime+margin]);
            title(eulerToPlot{i})
            xlabel('Time [s]')
            ylabel(eulerToPlot{i})
        end
    end
    
    % PLOT QUATERNION NORMALIZATION ERROR
    if plotQuatNormError
        quatNorm = sqrt(totalTrajectory.states(2,:).^2 + totalTrajectory.states(3,:).^2 + totalTrajectory.states(4,:).^2 + totalTrajectory.states(5,:).^2);
        eulerToPlot = {'Quaternion norm'};
        figure
        hold on
        plot(totalTrajectory.time,quatNorm)
        for j = 1:numel(WP_time)
            xval = WP_time(j);
            ymin = min(quatNorm(WP_index(j)),0);
            ymax = max(quatNorm(WP_index(j)),0);
            xPoints = [xval,xval];
            yPoints = [ymin,ymax];
            plot(xPoints,yPoints,'r--')
            text(xval,ymax,num2str(j),'Color','r','VerticalAlignment','bottom')
        end
        hold off
        grid
        margin = totalTrajectory.totalTime/20; % Lateral margins for x-axis in seconds
        xlim([-margin totalTrajectory.totalTime+margin]);
        title(eulerToPlot)
        xlabel('Time [s]')
        ylabel(eulerToPlot)
    end
    
    % PLOT ACCELERATIONS
    if plotAccels
        accelsToPlot = {'Acceleration module [m/s^2]' 'Acceleration z-axis [m/s^2]'};
        for i = 1:2
            figure
            hold on
            plot(totalTrajectory.time,totalTrajectory.accels(i,:))
            for j = 1:numel(WP_time)
                xval = WP_time(j);
                ymin = min(totalTrajectory.accels(i,WP_index(j)),0);
                ymax = max(totalTrajectory.accels(i,WP_index(j)),0);
                xPoints = [xval,xval];
                yPoints = [ymin,ymax];
                plot(xPoints,yPoints,'r--')
                text(xval,ymax,num2str(j),'Color','r','VerticalAlignment','bottom')
            end
            hold off
            grid
            margin = totalTrajectory.totalTime/20; % Lateral margins for x-axis in seconds
            xlim([-margin totalTrajectory.totalTime+margin]);
            title(accelsToPlot{i})
            xlabel('Time [s]')
            ylabel(accelsToPlot{i})
        end
    end
    
end

