    
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
    N = 250;
    estimatedTraj = cscvn([WP.ITA_north;WP.ITA_east;WP.ITA_up]);
    space = linspace(estimatedTraj.breaks(1),estimatedTraj.breaks(end),N);
    smooth = fnval(estimatedTraj,space);
    smooth_north = smooth(1,:)';
    smooth_east = smooth(2,:)';
    smooth_up = smooth(3,:)';

    % Create ITA figure
    ita_fig = figure('Visible','off','Resize','off','Position',[760,88,700,500]);  %  Create and then hide the UI as it is being constructed.
    ita_fig.NumberTitle = 'off'; % Deactivate the label "Figure n" in the title bar
    ita_fig.Name = 'Initial Trajectory Assistant'; % Assign the name to appear in the GUI title.
    movegui(ita_fig,'northeast') % Move the GUI.
    plot3D_ax = axes('Position',[0.1 0.25 0.7 0.7]);
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
%     hold off
    grid
%     title(['Estimated time ' num2str(propagatedState.totalTime) 's'])
    axis equal
    axis vis3d % Lock aspect ratio of axes
    view(-45,30); % Azimuth and elevation of initial view (degrees)
    set(plot3D_ax, 'Ydir', 'reverse')
    xlabel('North')
    ylabel('East')
    zlabel('Up')
%     set(ITA_WP,'Tag','plot3D_tag');
    
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
              'BackgroundColor',[0 1 0.5],...
              'String','Finish',...
              'Callback',@ITA_finishTraj_Callback);
    
    % Make GUI visible.
    ita_fig.Visible = 'on';
    
    function sliderNorthCallback(sliderNorthHandle,~)
        
        % Get slider value
        increment = get(sliderNorthHandle,'Value');
        
        % Update value in WP structure
        WP.ITA_north(WP.ITA_activeWP) = WP.ITA_north_orig(WP.ITA_activeWP) + increment;
        
        % Generate natural cubic spline through all new WP
        N = 250;
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
        
        % Update ITA_WP position
        set(ITA_WP{WP.ITA_activeWP},'XData',WP.ITA_north_orig(WP.ITA_activeWP) + increment);
        
    end

    function sliderEastCallback(sliderEastHandle,~)
        
        % Get slider value
        increment = get(sliderEastHandle,'Value');
        
        % Update value in WP structure
        WP.ITA_east(WP.ITA_activeWP) = WP.ITA_east_orig(WP.ITA_activeWP) + increment;
        
        % Generate natural cubic spline through all new WP
        N = 250;
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
        
        % Update ITA_WP position
        set(ITA_WP{WP.ITA_activeWP},'YData',WP.ITA_east_orig(WP.ITA_activeWP) + increment);
        
    end

    function sliderUpCallback(sliderUpHandle,~)
                
        % Deactivate limit auto adjustment
        
        
        % Get slider value
        increment = get(sliderUpHandle,'Value');
        
        % Update value in WP structure
        WP.ITA_up(WP.ITA_activeWP) = WP.ITA_up_orig(WP.ITA_activeWP) + increment;
        
        % Generate natural cubic spline through all new WP
        N = 250;
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
        
        % Update ITA_WP position
        set(ITA_WP{WP.ITA_activeWP},'ZData',WP.ITA_up_orig(WP.ITA_activeWP) + increment);
                    
        axis tight;
        
    end

    function ITA_previousWP_Callback(~,~)
        % Change active WP to previous
        set(ITA_WP{1},'MarkerFaceColor','b')
        set(ITA_WP{WP.ITA_activeWP},'MarkerFaceColor','b')
        if WP.ITA_activeWP_virtual == 1 % Go back to last
            WP.ITA_activeWP_virtual = WP.numOfVirtualWP_ITA;
        else
            WP.ITA_activeWP_virtual = WP.ITA_activeWP_virtual - 1;
        end
        WP.ITA_activeWP = WP.ITA_virtualWPindices(WP.ITA_activeWP_virtual);
        set(ITA_WP{WP.ITA_activeWP},'MarkerFaceColor','g')
    end

    function ITA_nextWP_Callback(~,~)
        % Change active WP to next
        set(ITA_WP{1},'MarkerFaceColor','b')
        set(ITA_WP{WP.ITA_activeWP},'MarkerFaceColor','b')
        if WP.ITA_activeWP_virtual == WP.numOfVirtualWP_ITA % Go back to first
            WP.ITA_activeWP_virtual = 1;
        else
            WP.ITA_activeWP_virtual = WP.ITA_activeWP_virtual + 1;
        end
        WP.ITA_activeWP = WP.ITA_virtualWPindices(WP.ITA_activeWP_virtual);
        set(ITA_WP{WP.ITA_activeWP},'MarkerFaceColor','g')
    end
    
end
