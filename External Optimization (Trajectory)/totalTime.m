
function [time] = totalTime(params,WP)

    segment.bounds.initialTime.low = 0;
    segment.bounds.initialTime.upp = 0;
    segment.bounds.finalTime.low = 0.5;
    segment.bounds.finalTime.upp = 100;

    segment.bounds.state.low = [0; 40; -0.5; -1e6; -1e6; -1e6; -1e6; -1e6; -1e6; -1e6];
    segment.bounds.state.upp = [9000; 1e6; 0.5; 1e6; 1e6; 1e6; 1e6; 1e6; 1e6; 1e6];
    segment.bounds.initialState.low = [4000; 80; 0.1; 1; 0; 0; 0; 0; 0; 0];
    segment.bounds.initialState.upp = [4000; 80; 0.1; 1; 0; 0; 0; 0; 0; 0];
    segment.bounds.finalState.low = [0; 40; -0.5; 0; 0; 0; 1; 0; 0; 50];
    segment.bounds.finalState.upp = [9000; 1e6; 0.5; 0; 0; 0; 1; 0; 0; 50];

    segment.bounds.control.low = [-10; -9000; -7.3];
    segment.bounds.control.upp = [10; 9000; 7.3];
    
    segment.guess.time = [0 20];
    segment.guess.state = [[4000; 80; 0.1; 1; 0; 0; 0; 0; 0; 0]...
                           [5000; 100; 0.1; 0; 0; 0; 1; 0; 0; 50]];
    segment.guess.control = zeros(3,2);

    trajectory = optimizeSegment(segment);
    
    time
    
end

