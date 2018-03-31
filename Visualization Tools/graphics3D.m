function graphics3D(WP,totalTrajectory)

    % 3D Graphical representation
    f1 = figure('Visible','Off'); % Create and then hide figure as it is being constructed.
    movegui(f1,'northwest') % Move the GUI to the center of the screen.
    ax = axes;
    hold on
    plot3(totalTrajectory.states(8,:),totalTrajectory.states(9,:),totalTrajectory.states(10,:));
    scatter3(WP.north,WP.east,WP.up,9,'r','filled');
    for i = 1:WP.numOfWP
        text(WP.north(i),WP.east(i),WP.up(i),num2str(i-1),'Color','r','VerticalAlignment','bottom');
    end
    hold off
    grid
    title(['Total time ' num2str(totalTrajectory.totalTime) 's'])
    % [~,~] = legend('Original points'); % "[~,~]=" prevents the bug in R2015b (https://www.mathworks.com/support/bugreports/1283854)
    axis equal
    axis vis3d % Lock aspect ratio of axes
    % view(-45,45); % Azimuth and elevation of initial view (degrees)
    set(ax, 'Ydir', 'reverse')
    xlabel('North')
    ylabel('East')
    zlabel('Up')
    f1.Visible = 'on'; % Make figure visible.

end

