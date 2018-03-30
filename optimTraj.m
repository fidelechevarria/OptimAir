function optimTraj

    % optimTraj: Red Bull Air Race Team 26 Flight Trajectory Optimization Software
    % Author: Fidel Echevarria Corrales

    % IMPORTANT NOTE: Solve graphical issues for R2015b MATLAB Compiler
    % To solve this bug go to: https://www.mathworks.com/support/bugreports/1293244
    % and apply official workaround 1.
    
    % If executing in deployed mode make "My documents" the current directory
    if isdeployed
        myuser = getenv('USERPROFILE');
        mydocdir = [myuser '\Documents'];
        cd(mydocdir)
    end
    
    % Add folders to path
    addpath('Dynamic Model');
    addpath('External Optimization (Trajectory)');
    addpath('Flight Plans');
    addpath('FlightGear');
    addpath('Internal Optimization (Segments)');
    addpath('Simulink Models');
    addpath('Test Functions');
    
    % Create GUI 

    %  Create and then hide the UI as it is being constructed.
    f = figure('Visible','off','Resize','off','Position',[760,88,500,500]);
    f.MenuBar = 'none'; % Deactivate Menu Bar of the figure
    f.NumberTitle = 'off'; % Deactivate the label "Figure n" in the title bar

    % Assign the name to appear in the GUI title.
    f.Name = 'optimTraj - Flight Trajectory Optimization Tool';

    % Move the GUI to the center of the screen.
    movegui(f,'center')
    
    % Construct GUI components.
    
    % Create a tab group
    tabgp = uitabgroup(gcf,'Position',[.01 .1 .98 .89]);
    tab1 = uitab(tabgp,'Title','Flight Plan');
    tab2 = uitab(tabgp,'Title','Initial Conditions');
    tab3 = uitab(tabgp,'Title','Dynamic Model');
    tab4 = uitab(tabgp,'Title','Options');
    
    NWP_popup = uicontrol('Style', 'popup',...
           'Parent',tab1,...
           'String', {'6','7','8','9','10','11','12'},...
           'Position', [80 395 40 50],...
           'Callback',@NWP_Callback,...
           'Tag','NWP_popup');
    uicontrol('Style','Text',...
           'Parent',tab1,...
           'String',{'Number of' 'Waypoints'},...
           'Position',[10 420 60 30]);
    WP_table = uitable('Parent',tab1,...
        'Data',{[] [] [] [] [] [] [] [] false;[] [] [] [] [] [] [] [] false;...
        [] [] [] [] [] [] [] [] false;[] [] [] [] [] [] [] [] false;...
        [] [] [] [] [] [] [] [] false;[] [] [] [] [] [] [] [] false},...
        'ColumnName',{'Lat (º)','Lat ('')','Lat ('''')',...
        'Lon (º)','Lon ('')','Lon ('''')','Hdng (º)','Gate (S/D)','3D'},...
        'ColumnEditable',true,...
        'Position',[13 140 460 270],...
        'ColumnWidth',{40 40 40 40 40 45 55 60 25},...
        'CellEditCallback',@popupInTable,...
        'Tag','WP_table');
        function popupInTable(hObject,CellEditData)
             r = CellEditData.Indices(1);
             c = CellEditData.Indices(2);
             if (c == 8)&&((CellEditData.NewData ~= 'S') && (CellEditData.NewData ~= 'D') && (CellEditData.NewData ~= 's') && (CellEditData.NewData ~= 'd'))
                warningstring = 'Incorrect value. Enter ''S'' for Single gates or ''D'' for Double gates.';
                dlgname = 'Warning';
                warndlg(warningstring,dlgname)
             end
             if (CellEditData.NewData == 's')
                 hObject.Data{r,c} = 'S';
             elseif (CellEditData.NewData == 'd')
                 hObject.Data{r,c} = 'D';
             end
        end
    uicontrol('Parent',f,...
         'Units','pixels',...
         'Position',[420 15 70 25],...
         'String','Optimize FP',...
         'Callback',@optimize_Callback);
    uicontrol('Parent',tab1,...
         'Units','pixels',...
         'Position',[15 107 55 25],...
         'String','Save FP',...
         'Callback',@saveFP_Callback);
    uicontrol('Parent',tab1,...
         'Units','pixels',...
         'Position',[75 107 55 25],...
         'String','Load FP',...
         'Callback',@loadFP_Callback);
    uicontrol('Parent',tab1,...
         'Units','pixels',...
         'Position',[135 107 55 25],...
         'String','Clear FP',...
         'Callback',@clearFP_Callback);
    ita_panel = uipanel('Parent',tab2,...
         'Title','Initial Trajectory Assistant',...
         'Position',[.02 .7 .5 .29]);
    launchITA_ = uicontrol('Parent',ita_panel,...
         'Units','pixels',...
         'Position',[15 20 55 25],...
         'String','Launch',...
         'Callback',@launchITA,...
         'Tag','launchITA');        
    
    % Initialize GUI
    
    

    % Make GUI visible.
    
    f.Visible = 'on';

    % Callbacks
    
    function optimize_Callback(~,~)
        wpData = WP_table.Data;
        llaData = wpData(1:end,1:6);
        [north, east, up] = custom_lla2flat(llaData);
        hdngData = str2double(wpData(1:end,7));
        gateTypeData = wpData(1:end,8);
        expectedManoeuvreData = wpData(1:end,9);
        for i = 1:numel(expectedManoeuvreData)
            if expectedManoeuvreData{i} == true
                expectedManoeuvreData{i} = '3D';
            elseif expectedManoeuvreData{i} == false
                expectedManoeuvreData{i} = '2D';
            end
        end
        WP = struct('north', north, 'east', -east, 'up', up,...
            'heading', hdngData, 'gateType', gateTypeData, 'expectedManoeuvre', expectedManoeuvreData);
        % Optimize trajectory
        optimize(WP);
    end
    
    function NWP_Callback(Object,~)
        NWP = get(Object,'Value');
        dataCell = WP_table.Data;
        [activeRows,~] = size(dataCell);
        if (activeRows >= NWP+5)
            WP_table.Data = dataCell(1:NWP+5,:);
        else
            emptyDataCell = {[] [] [] [] [] [] [] [] false};
            emptyDataCellGroup = emptyDataCell;
            for i = 1: (NWP+5 - activeRows - 1)
                emptyDataCellGroup = [emptyDataCellGroup;emptyDataCell];
            end
            WP_table.Data = [dataCell;emptyDataCellGroup];
        end
    end

    function saveFP_Callback(~,~)
        if ~exist('Flight Plans','dir')
            mkdir('Flight Plans')
            cd([pwd '\Flight Plans'])
        else
            cd([pwd '\Flight Plans'])
        end
        WPdata = WP_table.Data;
        uisave('WPdata','myFlightPlan');
        cd ..
    end

    function loadFP_Callback(~,~)
        if (exist('Flight Plans','dir') && (isempty(strfind(pwd,'Flight Plans'))))
            cd([pwd '\Flight Plans'])
        end
        [file,path] = uigetfile('*.mat','Select a MAT file');
        if (file ~= 0)
            filename = fullfile(path,file);
            dataStruct = load(filename,'-mat');
            WP_table.Data = dataStruct.WPdata;
            [NWP,~] = size(dataStruct.WPdata);
            NWP_popup.Value = NWP - 5; % Change NWP popup menu value
        end
        if exist('Flight Plans','dir')
            cd ..
        end
    end

    function clearFP_Callback(~,~)
        NWP = NWP_popup.Value;
        emptyDataCell = {[] [] [] [] [] [] [] [] false};
        emptyDataCellGroup = emptyDataCell;
        for i = 1:(NWP+5 - 1)
            emptyDataCellGroup = [emptyDataCellGroup;emptyDataCell];
        end
        WP_table.Data = emptyDataCellGroup;
    end

    function launchITA(~,~)
        wpData = WP_table.Data;
        llaData = wpData(1:end,1:6);
        hdngData = str2double(wpData(1:end,7)');
        segmentTypeData = double(cell2mat(wpData(1:end,9)));
        [north, east, up] = custom_lla2flat(llaData);
        WP = struct('north', north, 'east', -east, 'up', up, 'segment_type', segmentTypeData);
        Launch_ITA(WP);
    end
    
    % PONER EN FUNCIÓN APARTE
    function Launch_ITA(WP)
        ita_fig = figure('Visible','off','Resize','off','Position',[760,88,700,500]);  %  Create and then hide the UI as it is being constructed.
        ita_fig.MenuBar = 'none'; % Deactivate Menu Bar of the figure
        ita_fig.NumberTitle = 'off'; % Deactivate the label "Figure n" in the title bar
        ita_fig.Name = 'Initial Trajectory Assistant'; % Assign the name to appear in the GUI title.
        movegui(ita_fig,'northeast') % Move the GUI.
        plot3D_ax = axes('Position',[0.1 0.25 0.7 0.7]);
        hold on
        plot3D = scatter3(plot3D_ax,WP.north,WP.east,WP.up,9,'b','filled')
        hold off
        grid
    %     title(['Total time ' num2str(propagatedState.totalTime) 's'])
        axis equal
        axis vis3d % Lock aspect ratio of axes
        view(-45,30); % Azimuth and elevation of initial view (degrees)
        xlabel('North')
        ylabel('East')
        zlabel('Up')
        set(plot3D,'Tag','plot3D_tag');
        datacursormode(ita_fig)
        dcm_obj = datacursormode(ita_fig);
        set(dcm_obj,'UpdateFcn',@customCursorUpdateFcn)
        % Standard Java JSlider (20px high if no ticks/labels, otherwise use 45px)
        jSlider1 = javax.swing.JSlider;
        javacomponent(jSlider1,[400,20,200,50]);
        set(jSlider1, 'Value',84, 'MajorTickSpacing',20, 'MinorTickSpacing',5,'PaintLabels',true, 'PaintTicks',true);  % with labels, no ticks
        hjSlider1 = handle(jSlider1, 'CallbackProperties');
        set(hjSlider1, 'StateChangedCallback', @hjSlider1Callback);  %alternative
        % Standard Java JSlider (20px high if no ticks/labels, otherwise use 45px)
        jSlider2 = javax.swing.JSlider;
        javacomponent(jSlider2,[640,20,50,200]);
        set(jSlider2, 'Value',84, 'MajorTickSpacing',20,'Orientation',jSlider2.VERTICAL, 'MinorTickSpacing',5, 'PaintLabels',true, 'PaintTicks',true);  % with labels, no ticks
        hjSlider2 = handle(jSlider2, 'CallbackProperties');
        set(hjSlider2, 'StateChangedCallback', @hjSlider2Callback);  %alternative
        ita_fig.Visible = 'on'; % Make GUI visible.
        function output_txt = customCursorUpdateFcn(~,event_obj)
            selectedPos = event_obj.Position; % Object containing event data structure
            for i = 1:numel(WP.north) % Find selected Way-Point ID
                if (WP.north(i) == selectedPos(1)) && (WP.east(i) == selectedPos(2)) && (WP.up(i) == selectedPos(3))
                    selectedWP = i;
                    break
                end
            end
            setappdata(launchITA_,'h',selectedWP);
            output_txt = 'Hola';   %Data cursor text
        end
        function hjSlider1Callback(hjSlider1,~)
            disp(get(hjSlider1,'Value'));
            disp(getappdata(launchITA_,'h'));
        end
        function hjSlider2Callback(hjSlider2,~)
            plot3D = findobj(get(gca,'Children'),'Tag','plot3D_tag')
            disp(get(hjSlider2,'Value'));
    %         set(ax,'XData',[1 get(hjSlider2,'Value') 3]);
        end
    end

end