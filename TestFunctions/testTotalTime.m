
clearvars
close all

load('WP_for_test.mat');

params = zeros(WP.numOfWP,1);

[f,c,ceq] = totalTime(params,WP);

