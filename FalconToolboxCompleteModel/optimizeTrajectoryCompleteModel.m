
function [f,c,ceq,totalTrajectory] = optimizeTrajectoryCompleteModel(WP,guess,configuration)

    segment = cell(WP.numOfWP-1,1);

    for i = 1:WP.numOfWP-1
        
        if i == 1 % Properties for first segment
            boundaries.phase1.initBoundsLow = [80;0;0;0;0;WP.heading(1);0;0;0;WP.north(1);WP.east(1);WP.up(1)];
            boundaries.phase1.initBoundsUpp = [80;0;0;0;0;WP.heading(1);0;0;0;WP.north(1);WP.east(1);WP.up(1)];
            boundaries.phase1.finalBoundsLow = [10;-50;-50;0;0;WP.heading(2);0;0;0;WP.north(2);WP.east(2);WP.up(2)];
            boundaries.phase1.finalBoundsUpp = [100;50;50;0;0;WP.heading(2);0;0;0;WP.north(2);WP.east(2);WP.up(2)];
        end

        segmentGuess.time = guess.time{i};
        segmentGuess.states = guess.states{i};
        segmentGuess.controls = guess.controls{i};
        segment{i} = optimizeSegmentCompleteModel(boundaries,segmentGuess,configuration);
        
        % Properties for the rest of the segments
        if i ~= WP.numOfWP-1 % Not necessary in the last iteration
            initStates = segment{i}.Phases(1).StateGrid.Values(:,end);
            boundaries.phase1.initBoundsLow = initStates;
            boundaries.phase1.initBoundsUpp = initStates;
            boundaries.phase1.finalBoundsLow = [10;-50;-50;0;0;WP.heading(i+2);0;0;0;WP.north(i+2);WP.east(i+2);WP.up(i+2)];
            boundaries.phase1.finalBoundsUpp = [100;50;50;0;0;WP.heading(i+2);0;0;0;WP.north(i+2);WP.east(i+2);WP.up(i+2)];
        end
        
    end
    
    totalTrajectory.totalTime = 0;
    totalTrajectory.time = [];
    totalTrajectory.states = [];
    totalTrajectory.controls = [];
    totalTrajectory.accels = [];
    for i = 1:WP.numOfWP-1
        totalTrajectory.time = [totalTrajectory.time segment{i}.Phases(1).RealTime+totalTrajectory.totalTime];
        totalTrajectory.totalTime = totalTrajectory.totalTime + segment{i}.Phases(1).FinalTime.Value;
        totalTrajectory.states = [totalTrajectory.states segment{i}.Phases(1).StateGrid.Values];
        totalTrajectory.controls = [totalTrajectory.controls segment{i}.Phases(1).ControlGrids.Values];
        totalTrajectory.accels = [totalTrajectory.accels segment{i}.Phases(1).PathConstraintFunctions(1).OutputGrid.Values];
    end
    totalTrajectory.segmentSize = numel(segment{1}.Phases(1).RealTime);
    totalTrajectory.numOfSegments = WP.numOfWP - 1;
    totalTrajectory.numOfPoints = numel(totalTrajectory.time);
    [totalTrajectory.numOfStates,~] = size(totalTrajectory.states);
    [totalTrajectory.numOfControls,~] = size(totalTrajectory.controls);
    totalTrajectory.euler = totalTrajectory.states(4:6,:);
    
    f = totalTrajectory.totalTime;
    c = [];
    ceq = [];
    
end

