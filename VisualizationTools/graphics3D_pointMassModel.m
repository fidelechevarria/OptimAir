function graphics3D_pointMassModel(WP,totalTrajectory,configuration)

    % Compute trajectory arclength and prepare 3D planes states
    planeSeparation = configuration.options.planeSeparation;
    planeScale = configuration.options.planeScaleFactor;
    arclength = zeros(totalTrajectory.numOfPoints-1,1);
    for i = 1:totalTrajectory.numOfPoints-1
        arclength(i) = sqrt((totalTrajectory.states(6,i+1)-totalTrajectory.states(6,i))^2+...
                            (totalTrajectory.states(7,i+1)-totalTrajectory.states(7,i))^2+...
                            (totalTrajectory.states(8,i+1)-totalTrajectory.states(8,i))^2);
    end
    cumulativeArcLength = cumsum(arclength);
    totalArcLength = cumulativeArcLength(end);
    numOfPlanes = floor(totalArcLength/planeSeparation);
    planePointIndex = [1];
    for i = 1:numOfPlanes
        [~,index] = min(abs(cumulativeArcLength-planeSeparation*i));
        planePointIndex = [planePointIndex index];
    end
    planePos = [totalTrajectory.states(6,planePointIndex)'...
                totalTrajectory.states(7,planePointIndex)'...
                totalTrajectory.states(8,planePointIndex)'];
    planeAtt = [totalTrajectory.euler(1,planePointIndex)'...
                totalTrajectory.euler(2,planePointIndex)'...
                totalTrajectory.euler(3,planePointIndex)'];
            
    % Compute arclength of the trajectory projection in the horizontal plane
    verticalLinesSeparation = configuration.options.vertLinesSeparation;
    arclengthHoriz = zeros(totalTrajectory.numOfPoints-1,1);
    for i = 1:totalTrajectory.numOfPoints-1
        arclengthHoriz(i) = sqrt((totalTrajectory.states(6,i+1)-totalTrajectory.states(6,i))^2+...
                                 (totalTrajectory.states(7,i+1)-totalTrajectory.states(7,i))^2);
    end
    cumulativeArcLengthHoriz = cumsum(arclengthHoriz);
    totalArcLengthHoriz = cumulativeArcLengthHoriz(end);
    numOfVerticalLines = floor(totalArcLengthHoriz/verticalLinesSeparation);
    verticalLinesPointIndex = [1];
    for i = 1:numOfVerticalLines
        [~,index] = min(abs(cumulativeArcLengthHoriz-verticalLinesSeparation*i));
        verticalLinesPointIndex = [verticalLinesPointIndex index];
    end
    verticalLinesPosHoriz = [totalTrajectory.states(6,verticalLinesPointIndex)'...
                             totalTrajectory.states(7,verticalLinesPointIndex)'];
    
    % 3D Graphical representation
    f1 = figure('Visible','Off'); % Create and then hide figure as it is being constructed.
    movegui(f1,'northwest') % Move the GUI to the center of the screen.
    ax = axes;
    hold on
    plot3(totalTrajectory.states(6,:),totalTrajectory.states(7,:),totalTrajectory.states(8,:));
    plot3(totalTrajectory.states(6,:),totalTrajectory.states(7,:),zeros(1,totalTrajectory.numOfPoints),'Color',[0 0.5 1]);
    scatter3(WP.north,WP.east,WP.up,9,'r','filled');
    for i = 1:WP.numOfWP
        text(WP.north(i),WP.east(i),WP.up(i),num2str(i),'Color','r','VerticalAlignment','bottom','FontSize',18);
    end
    insertPlaneObject(planePos,planeAtt,planeScale); % Insert 3D plane objects
    for i = 1:numOfVerticalLines
        zmin = min(totalTrajectory.states(8,verticalLinesPointIndex(i)),0);
        zmax = max(totalTrajectory.states(8,verticalLinesPointIndex(i)),0);
        plot3(verticalLinesPosHoriz(i,1)*ones(1,2),verticalLinesPosHoriz(i,2)*ones(1,2),[zmin zmax],'Color',[0 0.5 1])
    end
    for i = 1:WP.numOfWP % Plot arrows marking gates heading
        arrowLength = configuration.options.arrowLength;
        arrowLengthNorth = arrowLength*cos(WP.heading(i));
        arrowLengthEast = arrowLength*sin(WP.heading(i));
        posStartArrow = [WP.north(i) WP.east(i) WP.up(i)];
        posFinishArrow = [WP.north(i)+arrowLengthNorth WP.east(i)+arrowLengthEast WP.up(i)];
        mArrow3(posStartArrow,posFinishArrow,'color','red','stemWidth',1,'facealpha',0.5);
    end
    % Plot Safety Line Elements
    if configuration.SL.SL_north(1) ~= false
        for i = 1:numel(configuration.SL.SL_north)
            if i < 3
                scatter3(configuration.SL.SL_north(i),configuration.SL.SL_east(i),0,'r','filled');
                text(configuration.SL.SL_north(i),configuration.SL.SL_east(i),0,['A' num2str(i)],'Color','r','VerticalAlignment','bottom','FontSize',14);
            elseif i >= 3
                scatter3(configuration.SL.SL_north(i),configuration.SL.SL_east(i),0,'r','filled');
                text(configuration.SL.SL_north(i),configuration.SL.SL_east(i),0,['B' num2str(i-2)],'Color','r','VerticalAlignment','bottom','FontSize',14);
            end
        end
        plot3([configuration.SL.SL_north(1) configuration.SL.SL_north(2)],[configuration.SL.SL_east(1) configuration.SL.SL_east(2)],[0 0],'Color','r','LineWidth',1);
        if numel(configuration.SL.SL_north) == 4
            plot3([configuration.SL.SL_north(3) configuration.SL.SL_north(4)],[configuration.SL.SL_east(3) configuration.SL.SL_east(4)],[0 0],'Color','r','LineWidth',1);
        end
    end
    % Insert information in annotation
    dim = [0.75 0.68 0.3 0.3];
    str = {['Total time ' num2str(totalTrajectory.totalTime) ' s'],...
        ['Initial speed ' num2str(configuration.dynamics.initVel) ' m/s'],...
        ['Wind speed ' num2str(configuration.dynamics.windVel) ' m/s'],...
        ['Wind heading ' num2str(rad2deg(configuration.dynamics.windHeading)) 'º']};
    annotation('textbox',dim,'String',str,'FitBoxToText','on');
    hold off
    grid
    % [~,~] = legend('Original points'); % "[~,~]=" prevents the bug in R2015b (https://www.mathworks.com/support/bugreports/1283854)
    axis equal
    axis vis3d
    axis tight
    setAxes3DPanAndZoomStyle(zoom,gca,'camera')
    view(-45,30); % Azimuth and elevation of initial view (degrees)
    set(ax, 'Ydir', 'reverse')
    xlabel('North')
    ylabel('East')
    zlabel('Up')
    f1.Visible = 'on'; % Make figure visible.
    % Save figure
    cd OutputFiles;
    saveas(f1,'3D_Trajectory.fig');
    saveas(f1,'3D_Trajectory.png');
    cd ..;
    
end

