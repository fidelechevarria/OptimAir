clearvars
close all

load('WP_for_test.mat');

params = zeros(WP.numOfWP,1);

results(params,WP,0);

