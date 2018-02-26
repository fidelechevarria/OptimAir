function [V_new,phi,theta,L,D,alpha,lat_G,lon_G] = optimFP_3D_dynamic_model(V_old,arclength,T,latcurv,loncurv,theta)

rho = 1.225;
S = 9.1;
Cd0 = 0.03;
K = 0.06;
Cl0 = 0.5;
Clalpha = 5.73;
m = 703;
g = 9.81;

fun = @(Vmed) optimFP_3D_eqns(Vmed,Vi,arclength,T,rho,S,Cd0,K,m,g,latcurv,loncurv,theta);
Vmed = fsolve(fun,V_old);

% Solve the rest of variables
V_new = max(2*(Vmed-V_old)+V_old,1);
phi = acos(1/sqrt(V_new^2*latcurv/g+1));
Cl = (m*g/(cos(phi)*cos(theta))+m*Vmed^2*loncurv)/(0.5*rho*Vmed^2*S);
Cd = Cd0 + K*Cl^2;
L = 0.5*rho*V_new^2*S*Cl;
D = 0.5*rho*V_new^2*S*Cd;
alpha = (Cl-Cl0)/Clalpha;
lat_G = (V_new^2*latcurv)/g;
lon_G = (V_new^2*loncurv)/g;