function [x,fval,x_G,fval_G] = optimTraj_optimize(WP,hdngData)

%     % Perform initial optimization using a genetic algorithm
%     [x_G,fval_G,~,~,~,~] = optimTraj_genetic(WP,hdngData);
    
%     % Perform final optimization using gradient-descent
%     [x,fval,~,~,~,~,~] = optimTraj_fmincon(x_G,WP,hdngData);
    
    % Perform global optimization using multiStart fmincon in a parallel pool
    [x,fval] = optimTraj_multiStartParallel(WP,hdngData);

%     % Perform global optimization using pattern search
%     [x,fval] = optimTraj_patternSearch(WP,hdngData);
    
    % Show optimization results
    automaticFGlaunchIsActivated = 0;
    optimTraj_results(x,WP,automaticFGlaunchIsActivated);

end