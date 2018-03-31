
function [f,c,ceq,totalTrajectory] = totalTime(params,WP)

    trajectory = cell(WP.numOfWP-1,1);

    for i = 1:WP.numOfWP-1
        
        if i == 1 % Properties for first segment
            initThrustLow = 5000;
            initThrustUpp = initThrustLow;
            finalThrustLow = 0;
            finalThrustUpp = 9000;
            initVelocityLow = 80;
            initVelocityUpp = initVelocityLow;
            finalVelocityLow = 40;
            finalVelocityUpp = 200;
            initAlphaLow = 0.1;
            initAlphaUpp = initAlphaLow;
            finalAlphaLow = -0.7;
            finalAlphaUpp = 0.7;
            initAttitudeLow = eul2quat(deg2rad([WP.heading(i)+params(i) 0 0]))';
            initAttitudeUpp = initAttitudeLow;
            if isequal(WP.gateType{i+1},'D')
                finalAttitudeLow = eul2quat(deg2rad([WP.heading(i+1)+params(i+1) 0 0]))';
                finalAttitudeUpp = finalAttitudeLow;
            else
                finalAttitudeLow = [-1 -1 -1 -1]';
                finalAttitudeUpp = [1 1 1 1]';
            end
            initPosLow = [WP.north(i) WP.east(i) WP.up(i)]';
            initPosUpp = initPosLow;
            finalPosLow = [WP.north(i+1) WP.east(i+1) WP.up(i+1)]';
            finalPosUpp = finalPosLow;
        end
        
        segment.bounds.initialTime.low = 0;
        segment.bounds.initialTime.upp = 0;
        segment.bounds.finalTime.low = 0.1;
        segment.bounds.finalTime.upp = 100;

        segment.bounds.state.low = [0; 40; -0.7; -1; -1; -1; -1; -1e5; -1e5; 5];
        segment.bounds.state.upp = [9000; 200; 0.7; 1; 1; 1; 1; 1e5; 1e5; 2000];
        segment.bounds.initialState.low = [initThrustLow; initVelocityLow; initAlphaLow; initAttitudeLow; initPosLow];
        segment.bounds.initialState.upp = [initThrustUpp; initVelocityUpp; initAlphaUpp; initAttitudeUpp; initPosUpp];
        segment.bounds.finalState.low = [finalThrustLow; finalVelocityLow; finalAlphaLow; finalAttitudeLow; finalPosLow];
        segment.bounds.finalState.upp = [finalThrustUpp; finalVelocityUpp; finalAlphaUpp; finalAttitudeUpp; finalPosUpp];

        segment.bounds.control.low = [-10; -9000; -7.3];
        segment.bounds.control.upp = [10; 9000; 7.3];

        segment.guess.time = [0 5];
        segment.guess.state = [[5000; 100; 0.1; initAttitudeLow; initPosLow]...
                               [5000; 100; 0.1; initAttitudeLow; finalPosLow]];
        segment.guess.control = zeros(3,2);

        trajectory{i} = optimizeSegment(segment);
        
        % Properties for the rest of the segments
        if i ~= WP.numOfWP-1 % Not necessary in the last iteration
            initThrustLow = trajectory{i}.states(1,end);
            initThrustUpp = initThrustLow;
            finalThrustLow = 0;
            finalThrustUpp = 9000;
            initVelocityLow = trajectory{i}.states(2,end);
            initVelocityUpp = initVelocityLow;
            finalVelocityLow = 40;
            finalVelocityUpp = 200;
            initAlphaLow = trajectory{i}.states(3,end);
            initAlphaUpp = initAlphaLow;
            finalAlphaLow = -0.7;
            finalAlphaUpp = 0.7;
            initAttitudeLow = [trajectory{i}.states(4,end) trajectory{i}.states(5,end)...
                               trajectory{i}.states(6,end) trajectory{i}.states(7,end)]';
            initAttitudeUpp = initAttitudeLow;
            if isequal(WP.gateType{i+2},'D')
                finalAttitudeLow = eul2quat(deg2rad([WP.heading(i+2)+params(i+2) 0 0]))';
                finalAttitudeUpp = finalAttitudeLow;
            else
                finalAttitudeLow = [-1 -1 -1 -1]';
                finalAttitudeUpp = [1 1 1 1]';
            end
            initPosLow = [WP.north(i+1) WP.east(i+1) WP.up(i+1)]';
            initPosUpp = initPosLow;
            finalPosLow = [WP.north(i+2) WP.east(i+2) WP.up(i+2)]';
            finalPosUpp = finalPosLow;
        end
        
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

