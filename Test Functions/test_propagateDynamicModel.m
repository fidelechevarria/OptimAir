
clearvars
close all

north = [0 -71.8671900000000 -128.918283625363 -188 -218.453416166850 -258.127000000000 -312.296080858381 -450 -575.055541994591 -588 -439.983641012471 -390.543000000000 -344.294891113834 -309.125000000000 -285.220164488413 -216.984400000000 -70.7665996034953];
east = [0 120 203.314753884504 261.937500000000 287.794568517867 320 371.992783769589 530 503.781294597807 358.031300000000 236.543480973559 180.644500000000 91.2381998042622 -6.28125000000000 -61.1070659182956 -149.031300000000 -268.645810533891];
up = 20*ones(1,numel(north));

trajectory = cscvn([north;east;up]);

state_history = propagateDynamicModel(trajectory);

figure
hold on
fnplt(trajectory);
plot3(state_history.x,state_history.y,state_history.h);
hold off
grid
view(45,45)
axis equal
axis vis3d

figure
plot(cumsum(0.05*ones(500,1))',state_history.V)
grid
title('V')

figure
plot(cumsum(0.05*ones(500,1))',state_history.alpha_dot)
grid
title('alpha_dot')

figure
plot(cumsum(0.05*ones(500,1))',state_history.T_dot)
grid
title('T_dot')

figure
plot(cumsum(0.05*ones(500,1))',state_history.p)
grid
title('p')

figure
plot(cumsum(0.05*ones(500,1))',state_history.alpha)
grid
title('alpha')

figure
plot(cumsum(0.05*ones(500,1))',state_history.T)
grid
title('T')

figure
plot(cumsum(0.05*ones(500,1))',state_history.roll)
grid
title('roll')
