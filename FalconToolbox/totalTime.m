
function [f,c,ceq,totalTrajectory] = totalTime(WP,guess)

    segment = cell(WP.numOfWP-1,1);

    for i = 1:WP.numOfWP-1
        
        if i == 1 % Properties for first segment
            configuration.phase1.initBoundsLow = [5000;80;0.1;eul2quat([WP.heading(1) 0 0])';WP.north(1);WP.east(1);WP.up(1)];
            configuration.phase1.initBoundsUpp = [5000;80;0.1;eul2quat([WP.heading(1) 0 0])';WP.north(1);WP.east(1);WP.up(1)];
            configuration.phase1.finalBoundsLow = [0;40;-0.7;eul2quat([WP.heading(2) 0 0])';WP.north(2);WP.east(2);WP.up(2)];
            configuration.phase1.finalBoundsUpp = [9000;200;0.7;eul2quat([WP.heading(2) 0 0])';WP.north(2);WP.east(2);WP.up(2)];
        end

        segment{i} = optimizeTrajectory(configuration);
        
        % Properties for the rest of the segments
        if i ~= WP.numOfWP-1 % Not necessary in the last iteration
            initStates = segment{i}.Phases(1).StateGrid.Values(1:3,end);
            configuration.phase1.initBoundsLow = [initStates;eul2quat([WP.heading(i) 0 0])';WP.north(i);WP.east(i);WP.up(i)];
            configuration.phase1.initBoundsUpp = [initStates;eul2quat([WP.heading(i) 0 0])';WP.north(i);WP.east(i);WP.up(i)];
            configuration.phase1.finalBoundsLow = [0;40;-0.7;eul2quat([WP.heading(i+1) 0 0])';WP.north(i+1);WP.east(i+1);WP.up(i+1)];
            configuration.phase1.finalBoundsUpp = [9000;200;0.7;eul2quat([WP.heading(i+1) 0 0])';WP.north(i+1);WP.east(i+1);WP.up(i+1)];
        end
        
    end
    
    totalTrajectory.totalTime = 0;
    totalTrajectory.time = [];
    totalTrajectory.states = [];
    totalTrajectory.controls = [];
    for i = 1:WP.numOfWP-1
        totalTrajectory.totalTime = totalTrajectory.totalTime + segment{i}.Phases(1).FinalTime.Value;
        totalTrajectory.time = [totalTrajectory.time segment{i}.Phases(1).RealTime];
        totalTrajectory.states = [totalTrajectory.states segment{i}.Phases(1).StateGrid.Values];
        totalTrajectory.controls = [totalTrajectory.controls segment{i}.Phases(1).ControlGrids.Values];
    end
    totalTrajectory.segmentSize = numel(segment{1}.Phases(1).RealTime);
    totalTrajectory.numOfSegments = WP.numOfWP - 1;
    totalTrajectory.numOfPoints = numel(totalTrajectory.time);
    [totalTrajectory.numOfStates,~] = size(totalTrajectory.states);
    [totalTrajectory.numOfControls,~] = size(totalTrajectory.controls);
    totalTrajectory.euler = quat2eul(totalTrajectory.states(4:7,:)','ZYX')';
    
    f = totalTrajectory.totalTime;
    c = [];
    ceq = [];
    
end

