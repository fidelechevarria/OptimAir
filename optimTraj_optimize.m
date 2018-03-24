function [x,fval,x_G,fval_G] = optimTraj_optimize(WP,hdngData)

%     % Perform initial optimization using a genetic algorithm
%     [x_G,fval_G,~,~,~,~] = optimTraj_genetic(WP,hdngData);
    
    % Perform final optimization using gradient-descent
    x_G = 0;
    [x,fval,~,~,~,~,~] = optimTraj_fmincon(x_G,WP,hdngData);

%     % Perform global optimization using multiStart fmincon in a parallel pool
%     [x,fval] = optimTraj_multiStartParallel(WP,hdngData);

%     % Perform global optimization using pattern search
%     [x,fval] = optimTraj_patternSearch(WP,hdngData);
    
    % Show optimization results
    automaticFGlaunchIsActivated = 0;
    optimTraj_results(x,WP,automaticFGlaunchIsActivated);

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
%     [x,fval,exitflag,output,lambda,grad,hessian] = optimTraj_fmincon_seq(IP,WP,1,0)
%      
%     % Display final results
%     disp(char('',output.message)); % Display the reason why the algorithm stopped iterating
%     disp(char('','Last point: ','',num2str(x'))); % Display solution
%     disp(char('','Last point optimized value: ','',num2str(fval),'')); % Display solution's optimized value
%     %%%%%%%%%%%%%%%%%%%%

end