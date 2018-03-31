
function [f,c,ceq,totalTrajectory] = totalTime(params,WP)

    trajectory = cell(WP.numOfWP-1,1);

    for i = 1:WP.numOfWP-1
        
        initAttitude = eul2quat(deg2rad([WP.heading(i)+params(i) 0 0]))';
        finalAttitude = eul2quat(deg2rad([WP.heading(i+1)+params(i+1) 0 0]))';
        initPos = [WP.north(i) WP.east(i) WP.up(i)]';
        finalPos = [WP.north(i+1) WP.east(i+1) WP.up(i+1)]';
        
        segment.bounds.initialTime.low = 0;
        segment.bounds.initialTime.upp = 0;
        segment.bounds.finalTime.low = 0.5;
        segment.bounds.finalTime.upp = 100;

        segment.bounds.state.low = [0; 40; -0.5; -1e6; -1e6; -1e6; -1e6; -1e6; -1e6; -1e6];
        segment.bounds.state.upp = [9000; 1e6; 0.5; 1e6; 1e6; 1e6; 1e6; 1e6; 1e6; 1e6];
        segment.bounds.initialState.low = [4000; 80; 0.1; initAttitude; initPos];
        segment.bounds.initialState.upp = [4000; 80; 0.1; initAttitude; initPos];
        segment.bounds.finalState.low = [0; 40; -0.5; finalAttitude; finalPos];
        segment.bounds.finalState.upp = [9000; 1e6; 0.5; finalAttitude; finalPos];

        segment.bounds.control.low = [-10; -9000; -7.3];
        segment.bounds.control.upp = [10; 9000; 7.3];

        segment.guess.time = [0 10];
        segment.guess.state = [[4000; 80; 0.1; initAttitude; initPos]...
                               [4000; 80; 0.1; finalAttitude; finalPos]];
        segment.guess.control = zeros(3,2);

        trajectory{i} = optimizeSegment(segment);
        
    end
    
    totalTrajectory.totalTime = 0;
    totalTrajectory.time = [];
    totalTrajectory.states = [];
    totalTrajectory.controls = [];
    for i = 1:WP.numOfWP-1
        totalTrajectory.totalTime = totalTrajectory.totalTime + trajectory{i}.time(end);
        if i == 1
            totalTrajectory.time = [totalTrajectory.time trajectory{i}.time];
        else
            totalTrajectory.time = [totalTrajectory.time trajectory{i}.time+totalTrajectory.time(end)];
        end
        totalTrajectory.states = [totalTrajectory.states trajectory{i}.states];
        totalTrajectory.controls = [totalTrajectory.controls trajectory{i}.controls];
    end
    totalTrajectory.segmentSize = trajectory{1}.segmentSize;
    totalTrajectory.numOfSegments = WP.numOfWP - 1;
    totalTrajectory.numOfPoints = numel(totalTrajectory.time);
    [totalTrajectory.numOfStates,~] = size(totalTrajectory.states);
    [totalTrajectory.numOfControls,~] = size(totalTrajectory.controls);
    totalTrajectory.euler = quat2eul(totalTrajectory.states(4:7,:)','ZYX')';
    
    f = totalTrajectory.totalTime;
    c = [];
    ceq = [];
    
end

