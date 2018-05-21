    
function ITA_completeModel(WP,configuration,dir)
    
    % Generate new sequence of WP
    WP.ITA_WP_type = [];
    WP.ITA_north = [];
    WP.ITA_east = [];
    WP.ITA_up = [];
    for i = 1:WP.numOfWP
        if i == WP.numOfWP
            WP.ITA_WP_type = [WP.ITA_WP_type 0];
            WP.ITA_north = [WP.ITA_north WP.north(i)];
            WP.ITA_east = [WP.ITA_east WP.east(i)];
            WP.ITA_up = [WP.ITA_up WP.up(i)];
        elseif isequal(WP.expectedManoeuvre{i},'2D')
            WP.ITA_WP_type = [WP.ITA_WP_type 0 1];
            WP.ITA_north = [WP.ITA_north WP.north(i) (WP.north(i)+WP.north(i+1))/2];
            WP.ITA_east = [WP.ITA_east WP.east(i) (WP.east(i)+WP.east(i+1))/2];
            WP.ITA_up = [WP.ITA_up WP.up(i) (WP.up(i)+WP.up(i+1))/2];
        else
            WP.ITA_WP_type = [WP.ITA_WP_type 0 1 1 1]; % 0: Real WP; 1: Virtual WP
            WP.ITA_north = [WP.ITA_north WP.north(i)...
                WP.north(i)+(WP.north(i+1)-WP.north(i))/4.0...
                WP.north(i)+(WP.north(i+1)-WP.north(i))/2.0...
                WP.north(i)+3*(WP.north(i+1)-WP.north(i))/4];
            WP.ITA_east = [WP.ITA_east WP.east(i)...
                WP.east(i)+(WP.east(i+1)-WP.east(i))/4.0...
                WP.east(i)+(WP.east(i+1)-WP.east(i))/2.0...
                WP.east(i)+3*(WP.east(i+1)-WP.east(i))/4];
            WP.ITA_up = [WP.ITA_up WP.up(i)...
                WP.up(i)+(WP.up(i+1)-WP.up(i))/4.0...
                WP.up(i)+(WP.up(i+1)-WP.up(i))/2.0...
                WP.up(i)+3*(WP.up(i+1)-WP.up(i))/4];
        end
    end
    WP.numOfWP_ITA = numel(WP.ITA_north);
    WP.ITA_numOfSegments = WP.numOfWP_ITA - 1;
    WP.ITA_north_orig = WP.ITA_north;
    WP.ITA_east_orig = WP.ITA_east;
    WP.ITA_up_orig = WP.ITA_up;
    WP.ITA_virtualWPindices = find(WP.ITA_WP_type);
    WP.ITA_realWPindices = find(~WP.ITA_WP_type);
    WP.numOfVirtualWP_ITA = numel(WP.ITA_virtualWPindices);
    
    % Generate natural cubic spline through all WP
    generateTraj();
    N = 200;
    [smooth_north,smooth_east,smooth_up] = multiEvaluateSpline(estimatedTraj,N);
    
    % Compute arclength of the trajectory projection in the horizontal plane
    verticalLinesSeparation = 30;
    arclengthHoriz = zeros(N-1,1);
    for i = 1:N-1
        arclengthHoriz(i) = sqrt((smooth_north(i+1)-smooth_north(i))^2+...
                                 (smooth_east(i+1)-smooth_east(i))^2);
    end
    cumulativeArcLengthHoriz = cumsum(arclengthHoriz);
    totalArcLengthHoriz = cumulativeArcLengthHoriz(end);
    numOfVerticalLines = floor(totalArcLengthHoriz/verticalLinesSeparation);
    verticalLinesPointIndex = [1];
    for i = 1:numOfVerticalLines
        [~,index] = min(abs(cumulativeArcLengthHoriz-verticalLinesSeparation*i));
        verticalLinesPointIndex = [verticalLinesPointIndex index];
    end
    verticalLinesPosHoriz = [smooth_north(verticalLinesPointIndex)...
                             smooth_east(verticalLinesPointIndex)];
    
    % Create ITA figure
    ita_fig = figure('Visible','off','Resize','off','Position',[760,88,1000,500]);  %  Create and then hide the UI as it is being constructed.
    ita_fig.NumberTitle = 'off'; % Deactivate the label "Figure n" in the title bar
    ita_fig.Name = 'Initial Trajectory Assistant'; % Assign the name to appear in the GUI title.
    movegui(ita_fig,'northeast') % Move the GUI.
    plot3D_ax = axes('Units','pixels','Position',[71 126 490 350]);
    hold on
    trajPlot = plot3(smooth_north,smooth_east,smooth_up,'Color','b','LineWidth',1);
    j = 1;
    for i = 1:WP.numOfWP_ITA
        if WP.ITA_WP_type(i) == 0 % Real WP
            ITA_WP{i} = scatter3(WP.ITA_north(i),WP.ITA_east(i),WP.ITA_up(i),'r','filled');
            text(WP.ITA_north(i),WP.ITA_east(i),WP.ITA_up(i),num2str(j),'Color','r','VerticalAlignment','bottom','FontSize',18);
            j = j + 1;
        else % Virtual WP
            ITA_WP{i} = scatter3(WP.ITA_north(i),WP.ITA_east(i),WP.ITA_up(i),'b','filled');
        end
    end
    % Plot vertical lines and trajectory projection in the horizontal plane
    trajHorizPlot = plot3(smooth_north,smooth_east,zeros(1,N),'Color',[0 0.5 1]);
    for i = 1:numOfVerticalLines
        zmin = min(smooth_up(verticalLinesPointIndex(i)),0);
        zmax = max(smooth_up(verticalLinesPointIndex(i)),0);
        verticalLinesPlot{i} = plot3(verticalLinesPosHoriz(i,1)*ones(1,2),verticalLinesPosHoriz(i,2)*ones(1,2),[zmin zmax],'Color',[0 0.5 1]);
    end
    % Plot arrows marking gates heading
    for i = 1:WP.numOfWP
        arrowLength = 40;
        arrowLengthNorth = arrowLength*cos(WP.heading(i));
        arrowLengthEast = arrowLength*sin(WP.heading(i));
        posStartArrow = [WP.north(i) WP.east(i) WP.up(i)];
        posFinishArrow = [WP.north(i)+arrowLengthNorth WP.east(i)+arrowLengthEast WP.up(i)];
        mArrow3(posStartArrow,posFinishArrow,'color','red','stemWidth',1,'facealpha',0.5);
    end
    grid
%     title(['Estimated time ' num2str(propagatedState.optimizeTrajectoryCompleteModel) 's'])
    axis equal
    axis vis3d
    axis tight
    setAxes3DPanAndZoomStyle(zoom,gca,'camera')
    view(-45,30); % Azimuth and elevation of initial view (degrees)
    set(plot3D_ax, 'Ydir', 'reverse')
    xlabel('North')
    ylabel('East')
    zlabel('Up')
    rotate3d on
    
    % Store information for active WP
    WP.ITA_activeWP_virtual = 1;
    WP.ITA_activeWP = WP.ITA_virtualWPindices(WP.ITA_activeWP_virtual);
    set(ITA_WP{WP.ITA_activeWP},'MarkerFaceColor','g')
    
    % Standard Java JSlider (20px high if no ticks/labels, otherwise use 45px)
    sliderNorth = javax.swing.JSlider(-1000,1000);
    javacomponent(sliderNorth,[20,20,300,50]);
    set(sliderNorth, 'Value',0, 'MajorTickSpacing',200, 'MinorTickSpacing',50,'PaintLabels',true, 'PaintTicks',true);
    sliderNorthHandle = handle(sliderNorth, 'CallbackProperties');
    set(sliderNorthHandle, 'StateChangedCallback', @sliderNorthCallback);  %alternative
    
    % Standard Java JSlider (20px high if no ticks/labels, otherwise use 45px)
    sliderEast = javax.swing.JSlider(-1000,1000);
    javacomponent(sliderEast,[330,20,300,50]);
    set(sliderEast, 'Value',0, 'MajorTickSpacing',200, 'MinorTickSpacing',50,'PaintLabels',true, 'PaintTicks',true);
    sliderEastHandle = handle(sliderEast, 'CallbackProperties');
    set(sliderEastHandle, 'StateChangedCallback', @sliderEastCallback);  %alternative
    
    % Standard Java JSlider (20px high if no ticks/labels, otherwise use 45px)
    sliderUp = javax.swing.JSlider(-1000,1000);
    javacomponent(sliderUp,[640,20,55,300]);
    set(sliderUp, 'Value',0, 'MajorTickSpacing',200,'Orientation',sliderUp.VERTICAL, 'MinorTickSpacing',50, 'PaintLabels',true, 'PaintTicks',true);
    sliderUpHandle = handle(sliderUp, 'CallbackProperties');
    set(sliderUpHandle, 'StateChangedCallback', @sliderUpCallback);  %alternative
    
    % Create text for sliders
    uicontrol('Style','Text',...
              'Parent',ita_fig,...
              'String','N',...
              'ForegroundColor','b',...
              'FontSize',15,...
              'Position',[162 7 18 20]);
    uicontrol('Style','Text',...
              'Parent',ita_fig,...
              'String','E',...
              'ForegroundColor','b',...
              'FontSize',15,...
              'Position',[472 7 18 20]);
    uicontrol('Style','Text',...
              'Parent',ita_fig,...
              'String','U',...
              'ForegroundColor','b',...
              'FontSize',15,...
              'Position',[660 7 18 20]);
          
    % Create interactive buttons
    uicontrol('Parent',ita_fig,...
              'Units','pixels',...
              'Position',[640 470 55 25],...
              'String','Next WP',...
              'Callback',@ITA_nextWP_Callback);
    uicontrol('Parent',ita_fig,...
              'Units','pixels',...
              'Position',[565 470 70 25],...
              'String','Previous WP',...
              'Callback',@ITA_previousWP_Callback);
    uicontrol('Parent',ita_fig,...
              'Units','pixels',...
              'Position',[640 440 55 25],...
              'String','Save',...
              'Callback',@ITA_save_Callback);
    uicontrol('Parent',ita_fig,...
              'Units','pixels',...
              'Position',[640 410 55 25],...
              'String','Load',...
              'Callback',@ITA_load_Callback);
    uicontrol('Parent',ita_fig,...
              'Units','pixels',...
              'Position',[640 380 55 25],...
              'BackgroundColor',[0 1 0.5],...
              'String','Finish',...
              'Callback',@ITA_finish_Callback);
    
    % Define Row names for state table 
    j = 0;
    m = 0;
    rowNameCell = cell(WP.numOfWP_ITA,1);
    for i = 1:WP.numOfWP_ITA
        if WP.ITA_WP_type(i) == 0
            j = j + 1;
            rowNameCell{i} = num2str(j);
            m = 0;
        else
            m = m + 1;
            rowNameCell{i} = strcat(num2str(j),'.',num2str(m));
        end
    end
    
    % Create table for state table
    stateTable = uitable('Parent',ita_fig,...
              'Data',cell(WP.numOfWP_ITA,2),...
              'ColumnName',{'V (m/s)','Roll (º)'},...
              'ColumnEditable',true,...
              'Position',[710 10 280 480],...
              'ColumnWidth',{50 50},...
              'RowName',rowNameCell,...
              'ColumnFormat',repmat({'numeric'},1,2),...
              'Tag','statesTable');
              %'CellEditCallback',@popupInTable,...
              
    % Make GUI visible.
    ita_fig.Visible = 'on';
   
    function generateTraj()
        estimatedTraj = cscvn([WP.ITA_north;WP.ITA_east;WP.ITA_up]);
    end

    function [north,east,up] = multiEvaluateSpline(spline,numberOfPoints)
        space = linspace(spline.breaks(1),spline.breaks(end),numberOfPoints);
        points = fnval(spline,space);
        north = points(1,:)';
        east = points(2,:)';
        up = points(3,:)';
    end
    
    function sliderNorthCallback(sliderNorthHandle,~)
        % Utilizar las funciones robotics.Rate y waitfor con Matlab R2017b
        % para agilizar los cálculos de estas funciones.
        
        % Get slider value
        increment = get(sliderNorthHandle,'Value');
        
        % Update value in WP structure
        WP.ITA_north(WP.ITA_activeWP) = WP.ITA_north_orig(WP.ITA_activeWP) + increment;
              
        % Update all lines and plots
        updateLinesAndPlots()
        
        % Update ITA_WP position
        set(ITA_WP{WP.ITA_activeWP},'XData',WP.ITA_north_orig(WP.ITA_activeWP) + increment);
        
    end

    function sliderEastCallback(sliderEastHandle,~)
        
        % Get slider value
        increment = get(sliderEastHandle,'Value');
        
        % Update value in WP structure
        WP.ITA_east(WP.ITA_activeWP) = WP.ITA_east_orig(WP.ITA_activeWP) + increment;
        
        % Update all lines and plots
        updateLinesAndPlots()
        
        % Update ITA_WP position
        set(ITA_WP{WP.ITA_activeWP},'YData',WP.ITA_east_orig(WP.ITA_activeWP) + increment);
        
    end

    function sliderUpCallback(sliderUpHandle,~)
                    
        % Get slider value
        increment = get(sliderUpHandle,'Value');
        
        % Update value in WP structure
        WP.ITA_up(WP.ITA_activeWP) = WP.ITA_up_orig(WP.ITA_activeWP) + increment;
        
        % Update all lines and plots
        updateLinesAndPlots()
        
        % Update ITA_WP position
        set(ITA_WP{WP.ITA_activeWP},'ZData',WP.ITA_up_orig(WP.ITA_activeWP) + increment);
        
    end

    function updateLinesAndPlots()
        
        % Generate natural cubic spline through all new WP
        estimatedTraj = cscvn([WP.ITA_north;WP.ITA_east;WP.ITA_up]);
        [smooth_north,smooth_east,smooth_up] = multiEvaluateSpline(estimatedTraj,N);
        
        % Delete old trajectory plot
        delete(trajPlot)
        
        % Generate new trajectory plot
        trajPlot = plot3(smooth_north,smooth_east,smooth_up,'Color','b','LineWidth',1);
        
        % Generate new arclength of the trajectory projection in the horizontal plane
        for k = 1:N-1
            arclengthHoriz(k) = sqrt((smooth_north(k+1)-smooth_north(k))^2+...
                                     (smooth_east(k+1)-smooth_east(k))^2);
        end
        cumulativeArcLengthHoriz = cumsum(arclengthHoriz);
        totalArcLengthHoriz = cumulativeArcLengthHoriz(end);
        numOfVerticalLines = floor(totalArcLengthHoriz/verticalLinesSeparation);
        verticalLinesPointIndex = [1];
        for k = 1:numOfVerticalLines
            [~,index] = min(abs(cumulativeArcLengthHoriz-verticalLinesSeparation*k));
            verticalLinesPointIndex = [verticalLinesPointIndex index];
        end
        verticalLinesPosHoriz = [smooth_north(verticalLinesPointIndex)...
                                 smooth_east(verticalLinesPointIndex)];
        
        % Delete old trajectory projection plot
        delete(trajHorizPlot)
        
        % Generate new trajectory projection plot
        trajHorizPlot = plot3(smooth_north,smooth_east,zeros(1,N),'Color',[0 0.5 1]);
        
        % Delete and generate new vertical lines
        for k = 1:numel(verticalLinesPlot)
            delete(verticalLinesPlot{k})
        end
        for k = 1:numOfVerticalLines
            zmin = min(smooth_up(verticalLinesPointIndex(k)),0);
            zmax = max(smooth_up(verticalLinesPointIndex(k)),0);
            verticalLinesPlot{k} = plot3(verticalLinesPosHoriz(k,1)*ones(1,2),verticalLinesPosHoriz(k,2)*ones(1,2),[zmin zmax],'Color',[0 0.5 1]);
        end
        
    end

    function updateAllWP()
        for h = 1:WP.numOfVirtualWP_ITA
            aux_activeWP_virtual = h;
            aux_ITA_activeWP = WP.ITA_virtualWPindices(aux_activeWP_virtual);
            set(ITA_WP{aux_ITA_activeWP},'XData',WP.ITA_north(aux_ITA_activeWP));
            set(ITA_WP{aux_ITA_activeWP},'YData',WP.ITA_east(aux_ITA_activeWP));
            set(ITA_WP{aux_ITA_activeWP},'ZData',WP.ITA_up(aux_ITA_activeWP));
        end
        set(ITA_WP{WP.ITA_activeWP},'MarkerFaceColor','g');
    end

    function setSliderValues()
        sliderNorthValue = WP.ITA_north(WP.ITA_activeWP) - WP.ITA_north_orig(WP.ITA_activeWP);
        sliderEastValue = WP.ITA_east(WP.ITA_activeWP) - WP.ITA_east_orig(WP.ITA_activeWP);
        sliderUpValue = WP.ITA_up(WP.ITA_activeWP) - WP.ITA_up_orig(WP.ITA_activeWP);
        set(sliderNorthHandle,'Value',sliderNorthValue);
        set(sliderEastHandle,'Value',sliderEastValue);
        set(sliderUpHandle,'Value',sliderUpValue);
    end

    function ITA_previousWP_Callback(~,~)
        % Change active WP to previous
        set(ITA_WP{WP.ITA_virtualWPindices(1)},'MarkerFaceColor','b')
        set(ITA_WP{WP.ITA_activeWP},'MarkerFaceColor','b')
        if WP.ITA_activeWP_virtual == 1 % Go back to last
            WP.ITA_activeWP_virtual = WP.numOfVirtualWP_ITA;
        else
            WP.ITA_activeWP_virtual = WP.ITA_activeWP_virtual - 1;
        end
        WP.ITA_activeWP = WP.ITA_virtualWPindices(WP.ITA_activeWP_virtual);
        set(ITA_WP{WP.ITA_activeWP},'MarkerFaceColor','g')
        setSliderValues();
    end

    function ITA_nextWP_Callback(~,~)
        % Change active WP to next
        set(ITA_WP{WP.ITA_virtualWPindices(1)},'MarkerFaceColor','b')
        set(ITA_WP{WP.ITA_activeWP},'MarkerFaceColor','b')
        if WP.ITA_activeWP_virtual == WP.numOfVirtualWP_ITA % Go back to first
            WP.ITA_activeWP_virtual = 1;
        else
            WP.ITA_activeWP_virtual = WP.ITA_activeWP_virtual + 1;
        end
        WP.ITA_activeWP = WP.ITA_virtualWPindices(WP.ITA_activeWP_virtual);
        set(ITA_WP{WP.ITA_activeWP},'MarkerFaceColor','g')
        setSliderValues();
    end
    
    function ITA_save_Callback(~,~)
        cd(dir);
        if ~exist('InitialTrajectories','dir')
            mkdir('InitialTrajectories')
            cd([pwd '\InitialTrajectories'])
        else
            cd([pwd '\InitialTrajectories'])
        end
        trajData = WP;
        stateData = stateTable.Data;
        ITAdata = {trajData stateData};
        uisave('ITAdata','myInitialTrajectory');
        cd ..
    end

    function ITA_load_Callback(~,~)
        cd(dir);
        if (exist('InitialTrajectories','dir') && (~contains(pwd,'InitialTrajectories')))
            cd([pwd '\InitialTrajectories'])
        end
        [file,path] = uigetfile('*.mat','Select a MAT file');
        if (file ~= 0)
            filename = fullfile(path,file);
            dataStruct = load(filename,'-mat');
            trajData = dataStruct.ITAdata{1};
            stateData = dataStruct.ITAdata{2};
            if size(stateData,2) ~= 2
                warningstring = 'This data corresponds to different dynamic model.';
                dlgname = 'Warning';
                warndlg(warningstring,dlgname)
                cd ..
                return
            elseif isequal(trajData.north,WP.north)
                set(ITA_WP{WP.ITA_activeWP},'MarkerFaceColor','b');
                stateTable.Data = stateData;
                WP = trajData;
                updateLinesAndPlots();
                updateAllWP();
                setSliderValues()
            else
                warningstring = 'Trajectory does not fit with selected Flight Plan.';
                dlgname = 'Warning';
                warndlg(warningstring,dlgname)
                cd ..
                return
            end
        end
        if exist('InitialTrajectories','dir')
            cd ..
        end
    end

    function ITA_finish_Callback(~,~)
        cd(dir);
        guess = generateGuess();
        [~,~,~,totalTrajectory] = optimizeTrajectoryCompleteModel(WP,guess,configuration);
        graphics3D_completeModel(WP,totalTrajectory); % 3D Graphical representation
        graphics2D_completeModel(WP,totalTrajectory,configuration); % 2D Graphical representation
    end

    function arcLength = calculateArcLength(north,east,up)
        arcLength = zeros(1,numel(north)-1);
        for k = 1:numel(north)-1
            arcLength(k) = sqrt((north(k+1)-north(k))^2+(east(k+1)-east(k))^2+(up(k+1)-up(k))^2);
        end
    end

    function guess = generateGuess()
        
        numOfPointsSubsegmentGuess = 200;
        numOfPointsSegmentGuess = 6;
        subsegmentGuess = cell(WP.ITA_numOfSegments,1);
        velocityGuessSubsegment = cell(WP.ITA_numOfSegments,1);
        rollGuessSubsegment = cell(WP.ITA_numOfSegments,1);
        pitchGuessSubsegment = cell(WP.ITA_numOfSegments,1);
        headingGuessSubsegment = cell(WP.ITA_numOfSegments,1);
        northGuessSubsegment = cell(WP.ITA_numOfSegments,1);
        eastGuessSubsegment = cell(WP.ITA_numOfSegments,1);
        upGuessSubsegment = cell(WP.ITA_numOfSegments,1);
        timeGuessSubsegment = cell(WP.ITA_numOfSegments,1);
        stateIndeces = [];
        
        generateTraj();
        
        for s = 1:WP.ITA_numOfSegments
            subsegmentGuess{s} = fnbrk(estimatedTraj,s);
            [northGuessSubsegment{s},eastGuessSubsegment{s},upGuessSubsegment{s}] = multiEvaluateSpline(subsegmentGuess{s},numOfPointsSubsegmentGuess);
            temp_pitch = [];
            temp_heading = [];
            for t = 1:numOfPointsSubsegmentGuess-1
                temp_pitch = [temp_pitch atan2(upGuessSubsegment{s}(t+1)-upGuessSubsegment{s}(t),sqrt((northGuessSubsegment{s}(t+1)-northGuessSubsegment{s}(t))^2+(eastGuessSubsegment{s}(t+1)-eastGuessSubsegment{s}(t))^2))];
                temp_heading = [temp_heading atan2(eastGuessSubsegment{s}(t+1)-eastGuessSubsegment{s}(t),northGuessSubsegment{s}(t+1)-northGuessSubsegment{s}(t))];
            end
            pitchGuessSubsegment{s} = [temp_pitch temp_pitch(end)]';
            headingGuessSubsegment{s} = [temp_heading temp_heading(end)]';
            arcLengthGuessSubsegment{s} = calculateArcLength(northGuessSubsegment{s},eastGuessSubsegment{s},upGuessSubsegment{s});
            stateIndeces = [stateIndeces s*(numOfPointsSubsegmentGuess-1)];
        end
        
        arcLengthGuessMatrix = cell2mat(arcLengthGuessSubsegment);
        arcLengthGuessVector = reshape(arcLengthGuessMatrix,[],1);
        arcLengthGuessCumsum = [0;cumsum(arcLengthGuessVector)];
        arcLengthValuesForStates = [0;arcLengthGuessCumsum(stateIndeces)];
        
        stateData = cell2mat(stateTable.Data);
        velocityGuessInWP = stateData(:,1)';
        rollGuessInWP = deg2rad(stateData(:,2))';
        
        velocityGuessTotal = pchip(arcLengthValuesForStates,velocityGuessInWP,arcLengthGuessCumsum);
        rollGuessTotal = pchip(arcLengthValuesForStates,rollGuessInWP,arcLengthGuessCumsum);
        
        for s = 1:WP.ITA_numOfSegments
            velocityGuessSubsegment{s} = velocityGuessTotal((s-1)*(numOfPointsSubsegmentGuess-1)+1:s*(numOfPointsSubsegmentGuess-1));
            rollGuessSubsegment{s} = rollGuessTotal((s-1)*(numOfPointsSubsegmentGuess-1)+1:s*(numOfPointsSubsegmentGuess-1));
            
            velocityGuessSubsegment{s} = [velocityGuessSubsegment{s};velocityGuessSubsegment{s}(end)];
            
            rollGuessSubsegment{s} = [rollGuessSubsegment{s};rollGuessSubsegment{s}(end)];
            
            timeGuessSubsegment{s} = [arcLengthGuessSubsegment{s}';0]./velocityGuessSubsegment{s};
        end
        
        timeGuess = cell(WP.numOfSegments,1);
        velocityGuess = cell(WP.numOfSegments,1);
        rollGuess = cell(WP.numOfSegments,1);
        pitchGuess = cell(WP.numOfSegments,1);
        headingGuess = cell(WP.numOfSegments,1);
        northGuess = cell(WP.numOfSegments,1);
        eastGuess = cell(WP.numOfSegments,1);
        upGuess = cell(WP.numOfSegments,1);
        pGuess = cell(WP.numOfSegments,1);
        qGuess = cell(WP.numOfSegments,1);
        rGuess = cell(WP.numOfSegments,1);
        vxGuess = cell(WP.numOfSegments,1);
        vyGuess = cell(WP.numOfSegments,1);
        vzGuess = cell(WP.numOfSegments,1);
        daGuess = cell(WP.numOfSegments,1);
        deGuess = cell(WP.numOfSegments,1);
        drGuess = cell(WP.numOfSegments,1);
        dtGuess = cell(WP.numOfSegments,1);
        
        for k = 1:WP.numOfSegments
            timeGuessMatrix = cumsum(cell2mat(timeGuessSubsegment(WP.ITA_realWPindices(k):WP.ITA_realWPindices(k+1)-1)));
            velocityGuessMatrix = cell2mat(velocityGuessSubsegment(WP.ITA_realWPindices(k):WP.ITA_realWPindices(k+1)-1));
            rollGuessMatrix = cell2mat(rollGuessSubsegment(WP.ITA_realWPindices(k):WP.ITA_realWPindices(k+1)-1));
            pitchGuessMatrix = cell2mat(pitchGuessSubsegment(WP.ITA_realWPindices(k):WP.ITA_realWPindices(k+1)-1));
            headingGuessMatrix = cell2mat(headingGuessSubsegment(WP.ITA_realWPindices(k):WP.ITA_realWPindices(k+1)-1));
            northGuessMatrix = cell2mat(northGuessSubsegment(WP.ITA_realWPindices(k):WP.ITA_realWPindices(k+1)-1));
            eastGuessMatrix = cell2mat(eastGuessSubsegment(WP.ITA_realWPindices(k):WP.ITA_realWPindices(k+1)-1));
            upGuessMatrix = cell2mat(upGuessSubsegment(WP.ITA_realWPindices(k):WP.ITA_realWPindices(k+1)-1));
            timeGuessMatrixDiff = diff(timeGuessMatrix);
            pGuessMatrix = diff(rollGuessMatrix)./timeGuessMatrixDiff;
            qGuessMatrix = diff(pitchGuessMatrix)./timeGuessMatrixDiff;
            rGuessMatrix = diff(headingGuessMatrix)./timeGuessMatrixDiff;
            pGuessMatrix((isnan(pGuessMatrix))) = 0;
            qGuessMatrix((isnan(qGuessMatrix))) = 0;
            rGuessMatrix((isnan(rGuessMatrix))) = 0;
            pGuessMatrix = [pGuessMatrix;0];
            qGuessMatrix = [qGuessMatrix;0];
            rGuessMatrix = [rGuessMatrix;0];
            
            timeGuess{k} = reshape(timeGuessMatrix,[],1);
            velocityGuess{k} = reshape(velocityGuessMatrix,[],1);
            rollGuess{k} = reshape(rollGuessMatrix,[],1);
            pitchGuess{k} = reshape(pitchGuessMatrix,[],1);
            headingGuess{k} = reshape(headingGuessMatrix,[],1);
            northGuess{k} = reshape(northGuessMatrix,[],1);
            eastGuess{k} = reshape(eastGuessMatrix,[],1);
            upGuess{k} = reshape(upGuessMatrix,[],1);
            pGuess{k} = reshape(pGuessMatrix,[],1);
            qGuess{k} = reshape(qGuessMatrix,[],1);
            rGuess{k} = reshape(rGuessMatrix,[],1);
            
            velocityGuessTS{k} = timeseries(velocityGuess{k},timeGuess{k});
            rollGuessTS{k} = timeseries(rollGuess{k},timeGuess{k});
            pitchGuessTS{k} = timeseries(pitchGuess{k},timeGuess{k});
            headingGuessTS{k} = timeseries(headingGuess{k},timeGuess{k});
            northGuessTS{k} = timeseries(northGuess{k},timeGuess{k});
            eastGuessTS{k} = timeseries(eastGuess{k},timeGuess{k});
            upGuessTS{k} = timeseries(upGuess{k},timeGuess{k});
            pGuessTS{k} = timeseries(pGuess{k},timeGuess{k});
            qGuessTS{k} = timeseries(qGuess{k},timeGuess{k});
            rGuessTS{k} = timeseries(rGuess{k},timeGuess{k});

            auxlinspace = linspace(timeGuess{k}(1),timeGuess{k}(end),numOfPointsSegmentGuess);
            velocityGuessTS{k} = resample(velocityGuessTS{k},auxlinspace);
            rollGuessTS{k} = resample(rollGuessTS{k},auxlinspace);
            pitchGuessTS{k} = resample(pitchGuessTS{k},auxlinspace);
            headingGuessTS{k} = resample(headingGuessTS{k},auxlinspace);
            northGuessTS{k} = resample(northGuessTS{k},auxlinspace);
            eastGuessTS{k} = resample(eastGuessTS{k},auxlinspace);
            upGuessTS{k} = resample(upGuessTS{k},auxlinspace);
            pGuessTS{k} = resample(pGuessTS{k},auxlinspace);
            qGuessTS{k} = resample(pGuessTS{k},auxlinspace);
            rGuessTS{k} = resample(rGuessTS{k},auxlinspace);
            
            timeGuess{k} = linspace(timeGuess{k}(1),timeGuess{k}(end),numOfPointsSegmentGuess)';
            velocityGuess{k} = velocityGuessTS{k}.Data;
            rollGuess{k} = rollGuessTS{k}.Data;
            pitchGuess{k} = pitchGuessTS{k}.Data;
            headingGuess{k} = headingGuessTS{k}.Data;
            northGuess{k} = northGuessTS{k}.Data;
            eastGuess{k} = eastGuessTS{k}.Data;
            upGuess{k} = upGuessTS{k}.Data;
            pGuess{k} = pGuessTS{k}.Data;
            qGuess{k} = qGuessTS{k}.Data;
            rGuess{k} = rGuessTS{k}.Data;
            
            vxGuess{k} = velocityGuess{k};
            vyGuess{k} = zeros(numel(vxGuess{k}),1);
            vzGuess{k} = zeros(numel(vxGuess{k}),1);
            
            daGuess{k} = zeros(numel(vxGuess{k}),1);
            deGuess{k} = zeros(numel(vxGuess{k}),1);
            drGuess{k} = zeros(numel(vxGuess{k}),1);
            dtGuess{k} = zeros(numel(vxGuess{k}),1);
            
        end
        
        for segment = 1:WP.numOfSegments
            guess.time{segment} = timeGuess{segment}';
            guess.states{segment} = [vxGuess{segment} vyGuess{segment} vzGuess{segment}...
                           rollGuess{segment} pitchGuess{segment} headingGuess{segment}...
                           pGuess{segment} qGuess{segment} rGuess{segment}...
                           northGuess{segment} eastGuess{segment} upGuess{segment}]';
            guess.controls{segment} = [daGuess{segment} deGuess{segment} drGuess{segment} dtGuess{segment}]';
        end
    end

end
