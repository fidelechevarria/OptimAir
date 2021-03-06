function graphics3D_completeModel(WP,totalTrajectory)

    % Compute trajectory arclength and prepare 3D planes states
    planeSeparation = 50;
    planeScale = 4;
    arclength = zeros(totalTrajectory.numOfPoints-1,1);
    for i = 1:totalTrajectory.numOfPoints-1
        arclength(i) = sqrt((totalTrajectory.states(10,i+1)-totalTrajectory.states(10,i))^2+...
                            (totalTrajectory.states(11,i+1)-totalTrajectory.states(11,i))^2+...
                            (totalTrajectory.states(12,i+1)-totalTrajectory.states(12,i))^2);
    end
    cumulativeArcLength = cumsum(arclength);
    totalArcLength = cumulativeArcLength(end);
    numOfPlanes = floor(totalArcLength/planeSeparation);
    planePointIndex = [1];
    for i = 1:numOfPlanes
        [~,index] = min(abs(cumulativeArcLength-planeSeparation*i));
        planePointIndex = [planePointIndex index];
    end
    planePos = [totalTrajectory.states(10,planePointIndex)'...
                totalTrajectory.states(11,planePointIndex)'...
                totalTrajectory.states(12,planePointIndex)'];
    planeAtt = [totalTrajectory.euler(3,planePointIndex)'...
                totalTrajectory.euler(2,planePointIndex)'...
                totalTrajectory.euler(1,planePointIndex)'];
            
    % Compute arclength of the trajectory projection in the horizontal plane
    verticalLinesSeparation = 30;
    arclengthHoriz = zeros(totalTrajectory.numOfPoints-1,1);
    for i = 1:totalTrajectory.numOfPoints-1
        arclengthHoriz(i) = sqrt((totalTrajectory.states(10,i+1)-totalTrajectory.states(10,i))^2+...
                                 (totalTrajectory.states(11,i+1)-totalTrajectory.states(11,i))^2);
    end
    cumulativeArcLengthHoriz = cumsum(arclengthHoriz);
    totalArcLengthHoriz = cumulativeArcLengthHoriz(end);
    numOfVerticalLines = floor(totalArcLengthHoriz/verticalLinesSeparation);
    verticalLinesPointIndex = [1];
    for i = 1:numOfVerticalLines
        [~,index] = min(abs(cumulativeArcLengthHoriz-verticalLinesSeparation*i));
        verticalLinesPointIndex = [verticalLinesPointIndex index];
    end
    verticalLinesPosHoriz = [totalTrajectory.states(10,verticalLinesPointIndex)'...
                             totalTrajectory.states(11,verticalLinesPointIndex)'];
    
    % 3D Graphical representation
    f1 = figure('Visible','Off'); % Create and then hide figure as it is being constructed.
    movegui(f1,'northwest') % Move the GUI to the center of the screen.
    ax = axes;
    hold on
    plot3(totalTrajectory.states(10,:),totalTrajectory.states(11,:),totalTrajectory.states(12,:));
    plot3(totalTrajectory.states(10,:),totalTrajectory.states(11,:),zeros(1,totalTrajectory.numOfPoints),'Color',[0 0.5 1]);
    scatter3(WP.north,WP.east,WP.up,9,'r','filled');
    for i = 1:WP.numOfWP
        text(WP.north(i),WP.east(i),WP.up(i),num2str(i),'Color','r','VerticalAlignment','bottom','FontSize',18);
    end
    insertPlaneObject(planePos,planeAtt,planeScale); % Insert 3D plane objects
    for i = 1:numOfVerticalLines
        zmin = min(totalTrajectory.states(12,verticalLinesPointIndex(i)),0);
        zmax = max(totalTrajectory.states(12,verticalLinesPointIndex(i)),0);
        plot3(verticalLinesPosHoriz(i,1)*ones(1,2),verticalLinesPosHoriz(i,2)*ones(1,2),[zmin zmax],'Color',[0 0.5 1])
    end
    for i = 1:WP.numOfWP % Plot arrows marking gates heading
        arrowLength = 40;
        arrowLengthNorth = arrowLength*cos(WP.heading(i));
        arrowLengthEast = arrowLength*sin(WP.heading(i));
        posStartArrow = [WP.north(i) WP.east(i) WP.up(i)];
        posFinishArrow = [WP.north(i)+arrowLengthNorth WP.east(i)+arrowLengthEast WP.up(i)];
        mArrow3(posStartArrow,posFinishArrow,'color','red','stemWidth',1,'facealpha',0.5);
    end
    hold off
    grid
    title(['Total time ' num2str(totalTrajectory.totalTime) 's'])
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

end

