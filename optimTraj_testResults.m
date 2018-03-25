close all
clearvars

IP = [-71.86719 -188 -258.127 -450 -588 -390.543 -309.125 -216.9844... % North initial points
      120 261.9375 320 530 358.0313 180.6445 -6.28125 -149.0313]'; % East initial poits

% IP = [-70 -160 -260 -480 -514 -366 -314 -200.0... % North initial points
%             100 240 320 550 344 136 8 -150.0];   % East initial poits

WP.north = [0
    -128.918283625363
    -218.453416166850
    -312.296080858381
    -575.055541994591
    -439.983641012471
    -344.294891113834
    -285.220164488413
    -70.7665996034953];

WP.east = [0
    203.314753884504
    287.794568517867
    371.992783769589
    503.781294597807
    236.543480973559
    91.2381998042622
    -61.1070659182956
    -268.645810533891];

WP.up = [20
         20
         20
         20
         20
         20
         20
         20
         20];
     
WP.segment_type = [0
                   0
                   0
                   0
                   0
                   0
                   0
                   0
                   0];

optimTraj_results(IP,WP,0)

