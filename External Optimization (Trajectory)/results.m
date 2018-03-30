function results(params,WP,automaticFGlaunchIsActivated)

    % Optimize complete trajectory using optimal parameters
    [~,~,~,totalTrajectory] = totalTime(params,WP);

    % Graphics generation
    % 3D Graphical representation
    f1 = figure('Visible','Off'); % Create and then hide figure as it is being constructed.
    movegui(f1,'northwest') % Move the GUI to the center of the screen.
    hold on
    plot3(totalTrajectory.states(8,:),totalTrajectory.states(9,:),totalTrajectory.states(10,:));
    scatter3(WP.north,WP.east,WP.up,9,'b','filled');
    hold off
    grid
    title(['Total time ' num2str(totalTrajectory.totalTime) 's'])
    % [~,~] = legend('Original points'); % "[~,~]=" prevents the bug in R2015b (https://www.mathworks.com/support/bugreports/1283854)
    axis equal
    axis vis3d % Lock aspect ratio of axes
    % view(-45,45); % Azimuth and elevation of initial view (degrees)
    xlabel('North')
    ylabel('East')
    zlabel('Up')

    % 2D Graphical representation
%     plotStates(propagatedState,WP);

    % Make figure visible.
    f1.Visible = 'on';

    % FlightGear interface

    % FG_initlat = 37.657773; %[decimal degrees]
    % FG_initlon = -122.282163; %[decimal degrees]
    % FG_initalt = 10; % [meters]
    % FG_initpitch = 0; % [radians]
    % 
    % earthRadiusAtPoles = 6356725; % Do not modify this value
    % earthRadiusAtEquator = 6378137; % Do not modify this value
    % localEarthRadius = sqrt(((earthRadiusAtEquator^2*cos(FG_initlat))^2+...
    %     (earthRadiusAtPoles^2*sin(FG_initlat))^2)/...
    %     ((earthRadiusAtEquator*cos(FG_initlat))^2+...
    %     (earthRadiusAtPoles*sin(FG_initlat))^2)); % [meters]
    % 
    % FG_lat = timeseries(FG_initlat+((y_coord-y_coord(1))/localEarthRadius)*180/pi,time);
    % FG_lon = timeseries(FG_initlon+((x_coord-x_coord(1))/localEarthRadius)*180/pi,time);
    % FG_alt = timeseries(FG_initalt*ones(numel(time),1),time);
    % FG_pitch = timeseries(FG_initpitch*ones(numel(time),1),time);
    % FG_roll = timeseries(phi,time);
    % FG_yaw = timeseries(heading,time);
    % 
    % if automaticFGlaunchIsActivated == 1
    %     system('optimFP_runFG &')
    %     pause(30)
    %     sim('optimFP_simulate_flight')
    % end

end