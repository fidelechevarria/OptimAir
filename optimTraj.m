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
    
    % Add to path directory of Falcon toolbox
    if exist([matlabroot '\toolbox\falcon'],'dir')
        addpath([matlabroot '\toolbox\falcon']);
    elseif exist('C:\Users\fidel\Desktop\falcon','dir') % (UAV Nav MSI Laptop)
        addpath('C:\Users\fidel\Desktop\falcon')
    end
    
    dir = pwd; % Save initial directory root
    
    % Make directory OutputFiles if not existant
    if ~exist('OutputFiles','dir')
        mkdir('OutputFiles');
    end
    % Add folders to path
    addpath('FalconToolboxCompleteModel');
    addpath('FalconToolboxPointMassModel');
    addpath('FlightGear');
    addpath('FlightPlans');
    addpath('InitialTrajectories');
    addpath('InitialTrajectoryAssistant');
    addpath('MiscellaneousTools');
    addpath('OutputFiles');
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
    tab4 = uitab(tabgp,'Title','Development');
     
    uicontrol('Parent',f,...
         'Units','pixels',...
         'Position',[15 15 70 25],...
         'BackgroundColor',[1 0.65 0],...
         'String','Launch FG',...
         'Callback',@launchFG_Callback); 
    
    uicontrol('Parent',f,...
         'Units','pixels',...
         'Position',[90 15 80 25],...
         'BackgroundColor',[0.59 0.85 1],...
         'String','Simulate Model',...
         'Callback',@simulate_Callback);
     
    animateButton = uicontrol('Parent',f,...
         'Units','pixels',...
         'Position',[175 15 80 25],...
         'BackgroundColor',[0.59 0.85 1],...
         'String','Animate Result',...
         'Callback',@animate_Callback);
     
    uicontrol('Parent',f,...
         'Units','pixels',...
         'Position',[420 15 70 25],...
         'BackgroundColor',[0 1 0.5],...
         'String','Optimize FP',...
         'Callback',@optimize_Callback);
    
    NWP_popup = uicontrol('Style', 'popup',...
           'Parent',tab1,...
           'String', {'2','3','4','5','6','7','8','9','10','11','12',...
           '13','14','15','16','17','18'},...
           'Position', [50 345 40 50],...
           'Callback',@NWP_Callback,...
           'Value',5,...
           'Tag','NWP_popup');
    uicontrol('Style','Text',...
           'Parent',tab1,...
           'String','WP''s',...
           'Position',[15 360 30 30]);
    WP_table = uitable('Parent',tab1,...
        'Data',{[] [] [] [] [] [] [] [] false;[] [] [] [] [] [] [] [] false;...
        [] [] [] [] [] [] [] [] false;[] [] [] [] [] [] [] [] false;...
        [] [] [] [] [] [] [] [] false;[] [] [] [] [] [] [] [] false},...
        'ColumnName',{'Lat (º)','Lat ('')','Lat ('''')',...
        'Lon (º)','Lon ('')','Lon ('''')','Hdng (º)','Gate (S/D)','3D'},...
        'ColumnEditable',true,...
        'Position',[13 120 460 240],...
        'ColumnWidth',{40 40 40 40 40 45 55 60 25},...
        'CellEditCallback',@popupInTable,...
        'Tag','WP_table');
    SL_table = uitable('Parent',tab1,...
        'Data',{[] [] [] [] [] [];[] [] [] [] [] []},...
        'ColumnName',{'Lat (º)','Lat ('')','Lat ('''')',...
        'Lon (º)','Lon ('')','Lon ('''')'},...
        'RowName',{'A1';'A2'},...
        'ColumnEditable',true,...
        'Position',[13 13 460 95],...
        'ColumnWidth',{40 40 40 40 40 45},...
        'Enable','off',...
        'Tag','SL_table');
    uicontrol('Style','Text',...
           'Parent',tab1,...
           'String','Safety Lines',...
           'Position',[180 370 40 30]);
    SL_popup = uicontrol('Style', 'popup',...
           'Parent',tab1,...
           'String', {'0','1','2'},...
           'Position', [220 345 40 50],...
           'Callback',@SL_Callback,...
           'Value',1,...
           'Tag','SL_popup');
    uicontrol('Parent',tab1,...
         'Units','pixels',...
         'Position',[285 373 55 25],...
         'String','Save FP',...
         'Callback',@saveFP_Callback);
    uicontrol('Parent',tab1,...
         'Units','pixels',...
         'Position',[345 373 55 25],...
         'String','Load FP',...
         'Callback',@loadFP_Callback);
    uicontrol('Parent',tab1,...
         'Units','pixels',...
         'Position',[405 373 55 25],...
         'String','Clear FP',...
         'Callback',@clearFP_Callback);    
    DynamicModel_table = uitable('Parent',tab2,...
        'Data',{[];[];[];[];[];[];[];[];[]},...
        'ColumnName',{'Value'},...
        'RowName',{'m [kg]';'g [m/s^2]';...
        'rho [kg/m^3]';'S [m^2]';...
        'CL_0';'CL_alpha';'CD_0';'K';'CD_p';...
        'Max G''s [Total]';'Max G''s [Negative z-axis]';...
        'Min vel [m/s]';'Max vel [m/s]';'Min Pos North [m]';...
        'Max Pos North [m]';'Min Pos East [m]';'Max Pos East [m]';...
        'Min alt [m]';'Max alt [m]';'Min alpha [º]';'Max alpha [º]';...
        'Max thrust [N]';'Max Ang Vel x-axis [º/s]';'Init Earth Vel [m/s]';...
        'Wind Vel [m/s]';'Wind Heading [º]';'Wind Elevation [º]'},...
        'ColumnEditable',true,...
        'Position',[13 13 460 390],...
        'ColumnWidth',{40},...
        'CellEditCallback',@popupInTable,...
        'Tag','Options_table');
    Options_table = uitable('Parent',tab3,...
        'Data',{[];[];true;true},...
        'ColumnName',{'Value'},...
        'RowName',{'Plot States';'Plot Controls';'Max iterations per segment';...
        'Segment Discretization Points';'Min Segment Time [s]';...
        'Max Segment Time [s]';'Vertical Lines Separation [m]';...
        '3D Arrow Length [m]';'3D Plane Separation [m]';'3D Plane Scale Factor';...
        'ITA Spline Points';'ITA Subsegment Points';'ITA Segment Points'},...
        'ColumnEditable',true,...
        'Position',[13 13 460 390],...
        'ColumnWidth',{'auto'},...
        'CellEditCallback',@popupInTable,...
        'Tag','Options_table');
    modelPanel = uibuttongroup('Parent',tab4,...
         'Units','pixels',...
         'Title','Model',...
         'Position',[10 343 100 67]);
    pointMassButton = uicontrol('style','radiobutton',...
         'Parent',modelPanel,...
         'String','Point-Mass',...
         'Position',[7 30 195 15],...
         'Enable','off',...
         'Value',1);
    completeModelButton = uicontrol('style','radiobutton',...
         'Parent',modelPanel,...
         'String','Complete',...
         'Position',[7 10 195 15],...
         'Enable','off',...
         'Value',0);
    
    % Initialize GUI
    DynamicModel_table.Data = {'750';'9.8056';'1.225';'9.84';'0.1205';...
        '5.7';'0.0054';'0.18';'0.01';'10';'2';'40';'200';'-10000';...
        '10000';'-10000';'10000';'15';'1000';'-30';'30';'5000';'200';'80';...
        '0';'0';'0'};
    Options_table.Data = {true;true;'1000';'80';'0';'200';'30';'40';'50';'4';'200';'200';'6'};

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

    function launchFG_Callback(~,~)
        system('FlightGear\runFG &');
    end

    function simulate_Callback(~,~)
        model = 'freeSimulation_pointMassModel';
        open(model);
        configuration = createConfiguration();
        set_param([bdroot '/Parameters/initVel'],'Value',num2str(configuration.dynamics.initVel));
        set_param([bdroot '/Parameters/m'],'Value',num2str(configuration.dynamics.m));
        set_param([bdroot '/Parameters/g'],'Value',num2str(configuration.dynamics.g));
        set_param([bdroot '/Parameters/rho'],'Value',num2str(configuration.dynamics.rho));
        set_param([bdroot '/Parameters/S'],'Value',num2str(configuration.dynamics.S));
        set_param([bdroot '/Parameters/Clalpha'],'Value',num2str(configuration.dynamics.Clalpha));
        set_param([bdroot '/Parameters/K'],'Value',num2str(configuration.dynamics.K));
        set_param([bdroot '/Parameters/Cd0'],'Value',num2str(configuration.dynamics.Cd0));
        set_param([bdroot '/Parameters/Cl0'],'Value',num2str(configuration.dynamics.Cl0));
        set_param([bdroot '/Parameters/Cdp'],'Value',num2str(configuration.dynamics.Cdp));
        set_param([bdroot '/Parameters/maxThrust'],'Value',num2str(configuration.dynamics.maxThrust));
        set_param([bdroot '/Parameters/max_p'],'Value',num2str(configuration.dynamics.max_p));
        set_param([bdroot '/Parameters/maxAlpha'],'Value',num2str(configuration.dynamics.maxAlpha));
        set_param(model,'SimulationCommand','start');
    end

    function animate_Callback(~,~)
        if sum(~ismember({'FG_q0','FG_q1','FG_q2','FG_q3','FG_x','FG_y','FG_alt','FG_time'},evalin('base','who'))) == 0
            model = 'trajectorySimulation_pointMassModel';
            open(model);
            set_param(model,'SimulationCommand','start');
        else
            warningstring = 'Trajectory result not found in workspace. Run optimization to generate results.';
            dlgname = 'Warning';
            warndlg(warningstring,dlgname);
            return;
        end
    end

    function optimize_Callback(~,~)
        cd(dir);
        wpData = WP_table.Data;
        SL_data = SL_table.Data;
        checkForEmptyValuesWP = cellfun('isempty',wpData);
        if SL_popup.Value ~= 1
            checkForEmptyValuesSL = cellfun('isempty',SL_data);
        else
            checkForEmptyValuesSL = 0;
        end
        if any(any(checkForEmptyValuesWP)) || any(any(checkForEmptyValuesSL))
            warningstring = 'Empty fields remaining.';
            dlgname = 'Warning';
            warndlg(warningstring,dlgname)
            return
        else
            llaData = wpData(1:end,1:6);
            refData = wpData(1,1:6);
            [north, east, up] = custom_lla2flat(llaData,refData);
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
            if SL_popup.Value ~= 1
                [SL_north, SL_east, ~] = custom_lla2flat(SL_data,refData);
            else
                SL_north = false;
                SL_east = false;
            end
            WP = createWP(north,east,up,hdngData,gateTypeData,expectedManoeuvreData);
            configuration = createConfiguration(SL_north, SL_east);
            f.Visible = 'off';
            if pointMassButton.Value == 1
                ITA_pointMassModel(WP,configuration,dir,f); % Launch Initial Trajectory Assistant
            elseif completeModelButton.Value == 1
                ITA_completeModel(WP,configuration,dir); % Launch Initial Trajectory Assistant
            end
        end
    end
    
    function SL_Callback(Object,~)
        SL = get(Object,'Value');
        switch SL
            case 1
                SL_table.RowName = {'A1';'A2'};
                SL_table.Data = {[] [] [] [] [] [];[] [] [] [] [] []};
                SL_table.Enable = 'off';
            case 2
                SL_table.RowName = {'A1';'A2'};
                SL_table.Data = {[] [] [] [] [] [];[] [] [] [] [] []};
                SL_table.Enable = 'on';
            case 3
                SL_table.RowName = {'A1';'A2';...
                    'B1';'B2'};
                SL_table.Data = {[] [] [] [] [] [];[] [] [] [] [] [];...
                    [] [] [] [] [] [];[] [] [] [] [] []};
                SL_table.Enable = 'on';
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
        data.FP_data = WP_table.Data;
        data.SL_data = SL_table.Data;
        data.SL_popup = SL_popup.Value;
        uisave('data','myFlightPlan');
        cd ..
    end

    function loadFP_Callback(~,~)
        cd(dir);
        if (exist('FlightPlans','dir') && ~contains(pwd,'FlightPlans'))
            cd([pwd '\FlightPlans'])
        end
        [file,path] = uigetfile('*.mat','Select a MAT file');
        if (file ~= 0)
            filename = fullfile(path,file);
            dataStruct = load(filename,'-mat');
            WP_table.Data = dataStruct.data.FP_data;
            [NWP,~] = size(dataStruct.data.FP_data);
            NWP_popup.Value = NWP - 1; % Change NWP popup menu value
            SL_table.Data = dataStruct.data.SL_data;
            SL_popup.Value = dataStruct.data.SL_popup;
            switch dataStruct.data.SL_popup
                case 1
                    SL_table.RowName = {'A1';'A2'};
                    SL_table.Enable = 'off';
                case 2
                    SL_table.RowName = {'A1';'A2'};
                    SL_table.Enable = 'on';
                case 3
                    SL_table.RowName = {'A1';'A2';...
                        'B1';'B2'};
                    SL_table.Enable = 'on';
            end
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

    function configuration = createConfiguration(SL_north, SL_east)
        configuration.options.plotStates = Options_table.Data{1};
        configuration.options.plotControls = Options_table.Data{2};
        configuration.options.majIterLim = str2double(Options_table.Data{3});
        configuration.options.discretizationPoints = str2double(Options_table.Data{4});
        configuration.options.minSegmentTime = str2double(Options_table.Data{5});
        configuration.options.maxSegmentTime = str2double(Options_table.Data{6});
        configuration.options.vertLinesSeparation = str2double(Options_table.Data{7});
        configuration.options.arrowLength = str2double(Options_table.Data{8});
        configuration.options.planeSeparation = str2double(Options_table.Data{9});
        configuration.options.planeScaleFactor = str2double(Options_table.Data{10});
        configuration.options.ITA_splinePoints = str2double(Options_table.Data{11});
        configuration.options.ITA_subsegmentPoints = str2double(Options_table.Data{12});
        configuration.options.ITA_segmentPoints = str2double(Options_table.Data{13});
        configuration.SL.SL_north = SL_north;
        configuration.SL.SL_east = SL_east;
        configuration.dynamics.m = str2double(DynamicModel_table.Data{1});
        configuration.dynamics.g = str2double(DynamicModel_table.Data{2});
        configuration.dynamics.rho = str2double(DynamicModel_table.Data{3});
        configuration.dynamics.S = str2double(DynamicModel_table.Data{4});
        configuration.dynamics.Cl0 = str2double(DynamicModel_table.Data{5});
        configuration.dynamics.Clalpha = str2double(DynamicModel_table.Data{6});
        configuration.dynamics.Cd0 = str2double(DynamicModel_table.Data{7});
        configuration.dynamics.K = str2double(DynamicModel_table.Data{8});
        configuration.dynamics.Cdp = str2double(DynamicModel_table.Data{9});
        configuration.dynamics.maxG = str2double(DynamicModel_table.Data{10});
        configuration.dynamics.maxG_neg = str2double(DynamicModel_table.Data{11});
        configuration.dynamics.minVel = str2double(DynamicModel_table.Data{12});
        configuration.dynamics.maxVel = str2double(DynamicModel_table.Data{13});
        configuration.dynamics.minPosNorth = str2double(DynamicModel_table.Data{14});
        configuration.dynamics.maxPosNorth = str2double(DynamicModel_table.Data{15});
        configuration.dynamics.minPosEast = str2double(DynamicModel_table.Data{16});
        configuration.dynamics.maxPosEast = str2double(DynamicModel_table.Data{17});
        configuration.dynamics.minAlt = str2double(DynamicModel_table.Data{18});
        configuration.dynamics.maxAlt = str2double(DynamicModel_table.Data{19});
        configuration.dynamics.minAlpha = deg2rad(str2double(DynamicModel_table.Data{20}));
        configuration.dynamics.maxAlpha = deg2rad(str2double(DynamicModel_table.Data{21}));
        configuration.dynamics.maxThrust = str2double(DynamicModel_table.Data{22});
        configuration.dynamics.max_p = deg2rad(str2double(DynamicModel_table.Data{23}));
        configuration.dynamics.initVel = str2double(DynamicModel_table.Data{24});
        configuration.dynamics.windVel = str2double(DynamicModel_table.Data{25});
        configuration.dynamics.windHeading = deg2rad(str2double(DynamicModel_table.Data{26}));
        configuration.dynamics.windElevation = deg2rad(str2double(DynamicModel_table.Data{27}));
        % Calculate wind components
        configuration.dynamics.windVelocityEarth = calculateWindComponents(configuration);
        % Calculate Safety Line Parameters (Slope and offset)
        switch numel(configuration.SL.SL_north)
            case 2
                configuration.SL.active = 1;
                [configuration.SL.A_m,configuration.SL.A_n] = calcSafetyLineParams(configuration.SL.SL_north(1:2),configuration.SL.SL_east(1:2));
                configuration.SL.B_m = 1;
                configuration.SL.B_n = 1;
            case 4
                configuration.SL.active = 2;
                [configuration.SL.A_m,configuration.SL.A_n] = calcSafetyLineParams(configuration.SL.SL_north(1:2),configuration.SL.SL_east(1:2));
                [configuration.SL.B_m,configuration.SL.B_n] = calcSafetyLineParams(configuration.SL.SL_north(3:4),configuration.SL.SL_east(3:4));
            otherwise
                configuration.SL.active = 0;
                configuration.SL.A_m = 1;
                configuration.SL.A_n = 1;
                configuration.SL.B_m = 1;
                configuration.SL.B_n = 1;
        end
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

    function [m,n] = calcSafetyLineParams(north,east)
        m = (north(2)-north(1))/(east(2)-east(1));
        n = north(1)-m*east(1);
    end

end