function [D,Y,L,LL,MM,NN] = dyn_forcesMomentsWind(rho,V,S,b,c,Cd,Cy,Cl,Cll,Cmm,Cnn)
qd_times_S = 0.5.*rho.*V.^2.*S;
qd_times_S_times_b = qd_times_S.*b;
qd_times_S_times_c = qd_times_S.*c;
D = qd_times_S.*Cd;
Y = qd_times_S.*Cy;
L = qd_times_S.*Cl;
LL = qd_times_S_times_b.*Cll;
MM = qd_times_S_times_c.*Cmm;
NN = qd_times_S_times_b.*Cnn;
end

