function [ curvature ] = optimTraj_ppcurv( smoothTraj, N )
% PPCURV Find curvature of piecewise-polynomial (ppform) in N points
% defined uniformly along the parameter of the curve

space = linspace(smoothTraj.breaks(1),smoothTraj.breaks(end),N);

firstDer = fnder(smoothTraj,1);
firstDerVal = fnval(firstDer,space);
firstDer_north = firstDerVal(1,:)';
firstDer_east = firstDerVal(2,:)';
firstDer_up = firstDerVal(3,:)';

secondDer = fnder(smoothTraj,2);
secondDerVal = fnval(secondDer,space);
secondDer_north = secondDerVal(1,:)';
secondDer_east = secondDerVal(2,:)';
secondDer_up = secondDerVal(3,:)';

curvature = sqrt((secondDer_up.*firstDer_east-secondDer_east.*firstDer_up).^2+(secondDer_north.*firstDer_up-secondDer_up.*firstDer_north).^2+(secondDer_east.*firstDer_north-secondDer_north.*firstDer_east).^2)./...
    ((firstDer_north.^2+firstDer_east.^2+firstDer_up.^2).^1.5);

end