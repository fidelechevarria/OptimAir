function [x,fval,x_G,fval_G] = optimize(WP,guess)

    params = zeros(WP.numOfWP,1);
    plotResults(params,WP,guess,0);

%     % Perform global optimization using multiStart fminbnd in a parallel pool
%     [x,fval] = multiStartParallel(WP);

%     % Perform initial optimization using a genetic algorithm
%     [x_G,fval_G,~,~,~,~] = genetic(WP);
    
%     % Perform final optimization using gradient-descent
%     [x,fval,~,~,~,~,~] = fminconstr(WP);

%     % Perform global optimization using pattern search
%     [x,fval] = patternSearch(WP);

%     % Show optimization results
%     automaticFGlaunchIsActivated = 0;
%     plotResults(x,WP,automaticFGlaunchIsActivated);

end