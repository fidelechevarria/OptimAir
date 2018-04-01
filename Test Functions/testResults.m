function testResults()

    clearvars
    close all

    load('WP_for_test.mat');

    params = zeros(WP.numOfWP,1);

    plotResults(params,WP,0);

end
