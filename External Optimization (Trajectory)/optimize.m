function [x,fval,x_G,fval_G] = optimize(WP)

    % Perform global optimization using multiStart fminbnd in a parallel pool
    [x,fval] = multiStartParallel(WP);

%     % Perform initial optimization using a genetic algorithm
%     [x_G,fval_G,~,~,~,~] = genetic(WP);
    
%     % Perform final optimization using gradient-descent
%     x_G = 0;
%     [x,fval,~,~,~,~,~] = fminconstr(x_G,WP);

%     % Perform global optimization using pattern search
%     [x,fval] = patternSearch(WP);
    
    

%     %%%%%%%%%%%%%%%%%%%% Sequential optimization
%     [numOfWaypoints,~] = size(WP.north);
%     
%     IP = [-70 -160 -260 -480 -514 -366 -314 -200.0... % North initial points
%             100 240 320 550 344 136 8 -150.0];   % East initial poits
%     
%     for i = 1:numOfWaypoints
%         
%     end
%     
%     [x,fval,exitflag,output,lambda,grad,hessian] = fminconstr_seq(IP,WP,1,0)
%      
%     % Display final results
%     disp(char('',output.message)); % Display the reason why the algorithm stopped iterating
%     disp(char('','Last point: ','',num2str(x'))); % Display solution
%     disp(char('','Last point optimized value: ','',num2str(fval),'')); % Display solution's optimized value
%     %%%%%%%%%%%%%%%%%%%%

    % Show optimization results
    automaticFGlaunchIsActivated = 0;
    results(x,WP,automaticFGlaunchIsActivated);

end