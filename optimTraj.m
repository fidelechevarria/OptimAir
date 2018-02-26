function optimTraj

% optimTraj: Red Bull Air Race Team 26 Flight Trajectory Optimization Software
    % Author: Fidel Echevarria Corrales

    %% IMPORTANT NOTE: Solve graphical issues for R2015b MATLAB Compiler
    % To solve this bug go to: https://www.mathworks.com/support/bugreports/1293244
    % and apply official workaround 1.
    
    %% If executing in deployed mode make "My documents" the current directory
    if isdeployed
        myuser = getenv('USERPROFILE');
        mydocdir = [myuser '\Documents'];
        cd(mydocdir)
    end
    
    %% Create GUI 

    %  Create and then hide the UI as it is being constructed.
    f = figure('Visible','off','Resize','off','Position',[760,88,500,500]);
    f.MenuBar = 'none'; % Deactivate Menu Bar of the figure
    f.NumberTitle = 'off'; % Deactivate the label "Figure n" in the title bar

    % Assign the name to appear in the GUI title.
    f.Name = 'optimTraj - Flight Trajectory Optimization Tool';

    % Move the GUI to the center of the screen.
    movegui(f,'center')
    
    %% Construct GUI components.
    
    % Create a tab group
    tabgp = uitabgroup(gcf,'Position',[.01 .01 .98 .98]);
    tab1 = uitab(tabgp,'Title','Flight Plan');
    tab2 = uitab(tabgp,'Title','Dynamic Model');
    tab3 = uitab(tabgp,'Title','Options');
    
    % Flight Plan tab
    NWP_popup = uicontrol('Style', 'popup',...
           'Parent',tab1,...
           'String', {'6','7','8','9','10','11','12'},...
           'Position', [80 395 40 50],...
           'Callback',@NWP_Callback);
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
        'CellEditCallback',@popupInTable);
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
    uicontrol('Parent',tab1,...
         'Units','pixels',...
         'Position',[260 107 70 25],...
         'String','Optimize FP',...
         'Callback',@optimize_Callback);
             
    
    
    
             
    %% Initialize GUI
    
    

    %% Make GUI visible.
    
    f.Visible = 'on';

    %% Callbacks
    
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
        if ~exist('optimTraj Flight Plans','dir')
            mkdir('optimTraj Flight Plans')
            cd([pwd '\optimTraj Flight Plans'])
        else
            cd([pwd '\optimTraj Flight Plans'])
        end
        WPdata = WP_table.Data;
        uisave('WPdata','myFlightPlan');
        cd ..
    end

    function loadFP_Callback(~,~)
        if (exist('optimTraj Flight Plans','dir') && (isempty(strfind(pwd,'optimTraj Flight Plans'))))
            cd([pwd '\optimTraj Flight Plans'])
        end
        [file,path] = uigetfile('*.mat','Select a MAT file');
        if (file ~= 0)
            filename = fullfile(path,file);
            dataStruct = load(filename,'-mat');
            WP_table.Data = dataStruct.WPdata;
            [NWP,~] = size(dataStruct.WPdata);
            NWP_popup.Value = NWP - 5; % Change NWP popup menu value
        end
        if exist('optimTraj Flight Plans','dir')
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

    function optimize_Callback(~,~)
        wpData = WP_table.Data;
        llaData = wpData(1:end,1:6);
        hdngData = str2double(wpData(1:end,7)');
        % Convert from geodetic latitude, longitude, and altitude to flat Earth position
        format long g
        ellipsoidModel = 'WGS84';
        [NWP,~] = size(llaData);
        lat_DMS = cell(1,NWP);
        lon_DMS = cell(1,NWP);
        lat = cell(1,NWP);
        lon = cell(1,NWP);
        flat = cell(1,NWP);
        north = [];
        east = [];
        up = [];
        for i = 1:NWP
            lat_DMS{1,i} = [str2double(llaData{i,1}) str2double(llaData{i,2}) str2double(llaData{i,3})];
            lon_DMS{1,i} = [str2double(llaData{i,4}) str2double(llaData{i,5}) str2double(llaData{i,6})];
            lat{1,i} =  dms2degrees(lat_DMS{1,i});
            lon{1,i} =  dms2degrees(lon_DMS{1,i});
            flat{1,i} = lla2flat([lat{1,i} lon{1,i} 20],[lat{1,1} lon{1,1}],0,0,ellipsoidModel);
            north = [north ; flat{1,i}(1)];
            east = [east ; flat{1,i}(2)];
            up = [up ; flat{1,i}(3)];
        end
        WP = struct('north', north, 'east', -east, 'up', up);
        % Optimize trajectory
        optimTraj_optimize(WP,hdngData)
    end
    
end