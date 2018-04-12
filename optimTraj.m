function optimTraj

    % optimTraj: Red Bull Air Race Team 26 Flight Trajectory Optimization Software
    % Developer: Fidel Echevarria Corrales

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
    addpath('Initial Trajectories');
    addpath('Initial Trajectory Assistant');
    addpath('Internal Optimization (Segments)');
    addpath('Simulink Models');
    addpath('Test Functions');
    addpath('Visualization Tools');
    
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
    tab2 = uitab(tabgp,'Title','Dynamic Model');
    tab3 = uitab(tabgp,'Title','Options');
    
    uicontrol('Parent',f,...
         'Units','pixels',...
         'Position',[420 15 70 25],...
         'String','Optimize FP',...
         'Callback',@optimize_Callback);
    
    NWP_popup = uicontrol('Style', 'popup',...
           'Parent',tab1,...
           'String', {'2','3','4','5','6','7','8','9','10','11','12',...
           '13','14','15','16','17','18'},...
           'Position', [80 345 40 50],...
           'Callback',@NWP_Callback,...
           'Value',5,...
           'Tag','NWP_popup');
    uicontrol('Style','Text',...
           'Parent',tab1,...
           'String',{'Number of' 'Waypoints'},...
           'Position',[10 370 60 30]);
    WP_table = uitable('Parent',tab1,...
        'Data',{[] [] [] [] [] [] [] [] false;[] [] [] [] [] [] [] [] false;...
        [] [] [] [] [] [] [] [] false;[] [] [] [] [] [] [] [] false;...
        [] [] [] [] [] [] [] [] false;[] [] [] [] [] [] [] [] false},...
        'ColumnName',{'Lat (º)','Lat ('')','Lat ('''')',...
        'Lon (º)','Lon ('')','Lon ('''')','Hdng (º)','Gate (S/D)','3D'},...
        'ColumnEditable',true,...
        'Position',[13 90 460 270],...
        'ColumnWidth',{40 40 40 40 40 45 55 60 25},...
        'CellEditCallback',@popupInTable,...
        'Tag','WP_table');
    uicontrol('Parent',tab1,...
         'Units','pixels',...
         'Position',[15 57 55 25],...
         'String','Save FP',...
         'Callback',@saveFP_Callback);
    uicontrol('Parent',tab1,...
         'Units','pixels',...
         'Position',[75 57 55 25],...
         'String','Load FP',...
         'Callback',@loadFP_Callback);
    uicontrol('Parent',tab1,...
         'Units','pixels',...
         'Position',[135 57 55 25],...
         'String','Clear FP',...
         'Callback',@clearFP_Callback);     
    
    % Initialize GUI
    
    

    % Make GUI visible.
    
    f.Visible = 'on';

    % Callbacks
    
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
    
    function optimize_Callback(~,~)
        wpData = WP_table.Data;
        checkForEmptyValues = cellfun('isempty',wpData);
        if any(any(checkForEmptyValues))
            warningstring = 'Empty fields remaining.';
            dlgname = 'Warning';
            warndlg(warningstring,dlgname)
            return
        else
            llaData = wpData(1:end,1:6);
            [north, east, up] = custom_lla2flat(llaData);
            hdngData = deg2rad(str2double(wpData(1:end,7)));
            gateTypeData = wpData(1:end,8);
            expectedManoeuvreData = wpData(1:end,9);
            for i = 1:numel(expectedManoeuvreData)
                if expectedManoeuvreData{i} == true
                    expectedManoeuvreData{i} = '3D';
                elseif expectedManoeuvreData{i} == false
                    expectedManoeuvreData{i} = '2D';
                end
            end
            WP.north = north;
            WP.east = east;
            WP.up = up;
            WP.heading = hdngData;
            WP.gateType = gateTypeData;
            WP.expectedManoeuvre = expectedManoeuvreData;
            WP.numOfWP = numel(north);
            WP.numOfSegments = WP.numOfWP - 1;
            ITA_main(WP); % Launch Initial Trajectory Assistant
        end
    end
    
    function NWP_Callback(Object,~)
        NWP = get(Object,'Value');
        dataCell = WP_table.Data;
        [activeRows,~] = size(dataCell);
        if (activeRows >= NWP+1)
            WP_table.Data = dataCell(1:NWP+1,:);
        else
            emptyDataCell = {[] [] [] [] [] [] [] [] false};
            emptyDataCellGroup = emptyDataCell;
            for i = 1: (NWP - activeRows)
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
            NWP_popup.Value = NWP - 1; % Change NWP popup menu value
        end
        if exist('Flight Plans','dir')
            cd ..
        end
    end

    function clearFP_Callback(~,~)
        NWP = NWP_popup.Value;
        emptyDataCell = {[] [] [] [] [] [] [] [] false};
        emptyDataCellGroup = emptyDataCell;
        for i = 1:(NWP)
            emptyDataCellGroup = [emptyDataCellGroup;emptyDataCell];
        end
        WP_table.Data = emptyDataCellGroup;
    end

end