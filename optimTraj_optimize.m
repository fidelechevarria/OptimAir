function [x,fval,x_G,fval_G] = optimTraj_optimize(WP,hdngData)

    % BORRAR %%%%%%%%%%%%%%%%%%%%%%%
    xLast = []; % Last place optimFP was called
    myf = []; % Use for objective at xLast

    % Define parameters
    funHandle = @trajApproxObjNested; % the objective function, nested below

    function y = trajApproxObjNested(x)
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf] = optimTraj_time(x,WP);
            xLast = x;
        end
        % Now compute objective function
        y = myf;
    end
    
    % Enter initial parameters for Newton-Raphson optimization
    x0 = [-70 -180 -260 -450 -500 -400 -310 -200.0... % North initial points
        100 230 320 450 350 150 50 -150.0... % East initial points
        20 20 20 20 20 20 20 20]; % Up initial points

    % Start Newton-Raphson optimization with the default options
    options = optimoptions('fminunc');

    % Modify optimization options setting
    options = optimoptions(options,'Display', 'off');
    options = optimoptions(options,'PlotFcns', {  @optimplotx @optimplotfval });
    options = optimoptions(options,'Diagnostics', 'off');

    % Run optimization
    [params,fval,~,output] = fminunc(funHandle,x0,options);

    % Display final results
    disp(char('',output.message)); % Display the reason why the algorithm stopped iterating
    disp(char('','Last point: ','',num2str(params'))); % Display solution
    disp(char('','Last point optimized value: ','',num2str(fval),'')); % Display solution's optimized value
    %%%%%%%%%%%%%%%%%%%%%%%

%     % Perform initial optimization using a genetic algorithm
%     [x_G,fval_G,~,~,~,~] = optimTraj_genetic(WP,hdngData);
%     
%     % Perform final optimization using gradient-descent
%     [x,fval,~,~,~,~,~] = optimTraj_newton(x_G,WP,hdngData);
    
    % Show optimization results
    x = 0; % BORRAR
    automaticFGlaunchIsActivated = 0;
    optimTraj_results(x,WP,automaticFGlaunchIsActivated);

end