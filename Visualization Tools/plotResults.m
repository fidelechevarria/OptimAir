function plotResults(params,WP,guess,automaticFGlaunchIsActivated)

    % Optimize complete trajectory using optimal parameters
    [~,~,~,totalTrajectory] = totalTime(params,WP,guess);

    % Graphics generation
    % 3D Graphical representation
    graphics3D(WP,totalTrajectory);
    
    % 2D Graphical representation
	graphics2D(WP,totalTrajectory);

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