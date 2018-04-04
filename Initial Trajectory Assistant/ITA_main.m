    
function ITA_main(WP)
    
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
    WP.ITA_north_orig = WP.ITA_north;
    WP.ITA_east_orig = WP.ITA_east;
    WP.ITA_up_orig = WP.ITA_up;
    WP.ITA_virtualWPindices = find(WP.ITA_WP_type);
    WP.numOfVirtualWP_ITA = numel(WP.ITA_virtualWPindices);
    
    % Generate natural cubic spline through all WP
    N = 200;
    estimatedTraj = cscvn([WP.ITA_north;WP.ITA_east;WP.ITA_up]);
    space = linspace(estimatedTraj.breaks(1),estimatedTraj.breaks(end),N);
    smooth = fnval(estimatedTraj,space);
    smooth_north = smooth(1,:)';
    smooth_east = smooth(2,:)';
    smooth_up = smooth(3,:)';

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
        mArrow3(posStartArrow,posFinishArrow,'color','red','stemWidth',1,'facealpha',0.5)
    end
    grid
%     title(['Estimated time ' num2str(propagatedState.totalTime) 's'])
    axis equal;
    axis vis3d; % Lock aspect ratio of axes
    axis tight;
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
              'Data',cell(WP.numOfWP_ITA,4),...
              'ColumnName',{'T (N)','V (m/s)','AoA (º)','Roll (º)'},...
              'ColumnEditable',true,...
              'Position',[710 10 280 480],...
              'ColumnWidth',{50 50 50 50},...
              'RowName',rowNameCell,...
              'ColumnFormat',repmat({'numeric'},1,4),...
              'Tag','statesTable');
              %'CellEditCallback',@popupInTable,...
              
    % Make GUI visible.
    ita_fig.Visible = 'on';
   
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
        space = linspace(estimatedTraj.breaks(1),estimatedTraj.breaks(end),N);
        smooth = fnval(estimatedTraj,space);
        smooth_north = smooth(1,:)';
        smooth_east = smooth(2,:)';
        smooth_up = smooth(3,:)';
        
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
        if ~exist('Initial Trajectories','dir')
            mkdir('Initial Trajectories')
            cd([pwd '\Initial Trajectories'])
        else
            cd([pwd '\Initial Trajectories'])
        end
        trajData = WP;
        stateData = stateTable.Data;
        ITAdata = {trajData stateData};
        uisave('ITAdata','myInitialTrajectory');
        cd ..
    end

    function ITA_load_Callback(~,~)
        if (exist('Initial Trajectories','dir') && (isempty(strfind(pwd,'Initial Trajectories'))))
            cd([pwd '\Initial Trajectories'])
        end
        [file,path] = uigetfile('*.mat','Select a MAT file');
        if (file ~= 0)
            filename = fullfile(path,file);
            dataStruct = load(filename,'-mat');
            trajData = dataStruct.ITAdata{1};
            stateData = dataStruct.ITAdata{2};
            if isequal(trajData.north,WP.north)
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
        if exist('Initial Trajectories','dir')
            cd ..
        end
    end

    function ITA_finish_Callback(~,~)
        
    end

end
