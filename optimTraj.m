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
    
    % Add to path directory of Falcon toolbox
    if exist([matlabroot '\toolbox\falcon'],'dir')
        addpath([matlabroot '\toolbox\falcon']);
    elseif exist('C:\Users\fidel\Desktop\falcon','dir') % (UAV Nav MSI Laptop)
        addpath('C:\Users\fidel\Desktop\falcon')
    end
    
    dir = pwd; % Save initial directory root
    
    % Add folders to path
    addpath('FalconToolboxCompleteModel');
    addpath('FalconToolboxPointMassModel');
    addpath('FlightGear');
    addpath('FlightPlans');
    addpath('InitialTrajectories');
    addpath('InitialTrajectoryAssistant');
    addpath('MiscellaneousTools');
    addpath('SimulinkModels');
    addpath('TestFunctions');
    addpath('VisualizationTools');
    
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
    
     Options_table = uitable('Parent',tab3,...
        'Data',{[];[];true;true},...
        'ColumnName',{'Value'},...
        'RowName',{'Max iterations per segment';'Discretization points per segment';...
        'Plot States';'Plot Controls'},...
        'ColumnEditable',true,...
        'Position',[13 13 460 390],...
        'ColumnWidth',{'auto'},...
        'CellEditCallback',@popupInTable,...
        'Tag','Options_table');
    modelPanel = uibuttongroup('Parent',tab2,...
         'Units','pixels',...
         'Title','Model',...
         'Position',[10 343 100 67]);
    pointMassButton = uicontrol('style','radiobutton',...
         'Parent',modelPanel,...
         'String','Point-Mass',...
         'Position',[7 30 195 15],...
         'Value',1);
    completeModelButton = uicontrol('style','radiobutton',...
         'Parent',modelPanel,...
         'String','Complete',...
         'Position',[7 10 195 15],...
         'Value',0);
    
    % Initialize GUI
    Options_table.Data = {'500';'80';true;true};

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
            WP = createWP(north,east,up,hdngData,gateTypeData,expectedManoeuvreData);
            configuration = createConfiguration();
            if pointMassButton.Value == 1
                ITA_pointMassModel(WP,configuration,dir); % Launch Initial Trajectory Assistant
            elseif completeModelButton.Value == 1
                ITA_completeModel(WP,configuration,dir); % Launch Initial Trajectory Assistant
            end
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
        cd(dir);
        if ~exist('FlightPlans','dir')
            mkdir('FlightPlans')
            cd([pwd '\FlightPlans'])
        else
            cd([pwd '\FlightPlans'])
        end
        WPdata = WP_table.Data;
        uisave('WPdata','myFlightPlan');
        cd ..
    end

    function loadFP_Callback(~,~)
        cd(dir);
        if (exist('FlightPlans','dir') && isempty(strfind(pwd,'FlightPlans')))
            cd([pwd '\FlightPlans'])
        end
        [file,path] = uigetfile('*.mat','Select a MAT file');
        if (file ~= 0)
            filename = fullfile(path,file);
            dataStruct = load(filename,'-mat');
            WP_table.Data = dataStruct.WPdata;
            [NWP,~] = size(dataStruct.WPdata);
            NWP_popup.Value = NWP - 1; % Change NWP popup menu value
        end
        if exist('FlightPlans','dir')
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

    function configuration = createConfiguration()
        configuration.majIterLim = str2double(Options_table.Data{1});
        configuration.discretizationPoints = str2double(Options_table.Data{2});
        configuration.plotStates = Options_table.Data{3};
        configuration.plotControls = Options_table.Data{4};
    end

    function WP = createWP(north,east,up,hdngData,gateTypeData,expectedManoeuvreData)
        WP.north = north;
        WP.east = east;
        WP.up = up;
        WP.heading = hdngData;
        WP.gateType = gateTypeData;
        WP.expectedManoeuvre = expectedManoeuvreData;
        WP.numOfWP = numel(north);
        WP.numOfSegments = WP.numOfWP - 1;
    end

end