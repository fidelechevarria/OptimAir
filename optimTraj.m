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
    tabgp = uitabgroup(gcf,'Position',[.01 .1 .98 .89]);
    tab1 = uitab(tabgp,'Title','Flight Plan');
    tab2 = uitab(tabgp,'Title','Initial Conditions');
    tab3 = uitab(tabgp,'Title','Dynamic Model');
    tab4 = uitab(tabgp,'Title','Options');
    
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
        'ColumnName',{'Lat (�)','Lat ('')','Lat ('''')',...
        'Lon (�)','Lon ('')','Lon ('''')','Hdng (�)','Gate (S/D)','3D'},...
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
    
    ita_panel = uipanel('Parent',tab2,'Title','Initial Trajectory Assistant',...
              'Position',[.02 .7 .5 .29]);
    
    uicontrol('Parent',ita_panel,...
         'Units','pixels',...
         'Position',[15 20 55 25],...
         'String',['Launch'],...
         'Callback',@launchITA);
 
             
    %% Initialize GUI
    
    

    %% Make GUI visible.
    
    f.Visible = 'on';

    %% Callbacks
    
    function optimize_Callback(~,~)
        wpData = WP_table.Data;
        llaData = wpData(1:end,1:6);
        hdngData = str2double(wpData(1:end,7)');
        segmentTypeData = double(cell2mat(wpData(1:end,9)));
        [north, east, up] = optimTraj_lla2flat(llaData);
        WP = struct('north', north, 'east', -east, 'up', up, 'segment_type', segmentTypeData);
        % Optimize trajectory
        optimTraj_optimize(WP,hdngData);
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

    function launchITA(~,~)
        wpData = WP_table.Data;
        llaData = wpData(1:end,1:6);
        hdngData = str2double(wpData(1:end,7)');
        segmentTypeData = double(cell2mat(wpData(1:end,9)));
        [north, east, up] = optimTraj_lla2flat(llaData);
        WP = struct('north', north, 'east', -east, 'up', up, 'segment_type', segmentTypeData);
        optimTraj_ITA(WP);
    end
    
end