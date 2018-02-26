
function [V_new,phi,L,D,alpha,lat_G] = optimTraj_model2D(V_old,alpha_old,arclength,T,latcurv)

rho = 1.225;
S = 9.1;
K = 0.1779;
m = 750;
g = 9.81;

% Look-up tables for Clalpha and Cd0
Clalpha_xdata = [-3.14 -2.36 -1.57 -0.78 -0.6 -0.28 -0.25 -0.22 -0.2 0 0.2 0.22 0.25 0.28 0.6 0.78 1.57 2.36 3.14];
Clalpha_ydata =  convforce([0.05 1.0 0.0 1.0 0.7 -1.12 -1.34 -1.4 -1.34 0.0 1.34 1.4 1.34 1.12 0.7 1.0 0.0 -1.0 0.05],'lbf','N');
Clalpha = fixpt_interp1(Clalpha_xdata,Clalpha_ydata,alpha_old,ufix(8),2^-8,sfix(16),2^-14,'Floor');

Cd0_xdata = [-3.142 -1.57 -0.26 0 0.26 1.57 3.142];
Cd0_ydata = convforce([0.06 1.5 0.036 0.028 0.036 1.5 0.06],'lbf','N');
Cd0 = fixpt_interp1(Cd0_xdata,Cd0_ydata,alpha_old,ufix(8),2^-8,sfix(16),2^-14,'Floor');

%%% MAX G-FORCE 10G;
%%% MAX SPEED 426 km/h = 118.33 m/s;
%%% MAX ROLL RATE 420 º/s;
%%% Vs = 31 m/s;

% Analytic equations:
%
% syms Vmed Vi arclength T rho S Cd0 K Cl0 Clalpha m g curv theta
% 
% cos_phi = 1/sqrt((Vmed^2*curv/g)^2+1); % NOTE: cos(atan(x)) = 1/sqrt(x^2+1);
% eqn = (2*(Vmed-Vi))/(arclength/Vmed)==(T-0.5*rho*Vmed^2*S*(Cd0+K*(m*g/(0.5*rho*Vmed^2*S*cos_phi))^2))/m;
% sol = solve(eqn,Vmed);
% pretty(sol)

% The problem of finding the Medium velocity at each timestep reduces to a 
% quartic function, which is solved using Ferrari's formula

% a*x^4+b*x^3+c*x^2+d*x+e=0;

a = 4*K*arclength*latcurv^2*m^2+Cd0*S^2*arclength*rho^2+4*S*m*rho;
b = -4*S*V_old*m*rho;
c = -2*S*T*arclength*rho;
d = 0;
e = 4*K*arclength*g^2*m^2;

discriminant = 256*a^3*e^3-192*a^2*b*d*e^2-128*a^2*c^2*e^2+144*a^2*c*d^2*e-27*a^2*d^4+...
               +144*a*b^2*c*e^2-6*a*b^2*d^2*e-80*a*b*c^2*d*e+18*a*b*c*d^3+16*a*c^4*e-...
               4*a*c^3*d^2-27*b^4*e^2+18*b^3*c*d*e-4*b^3*d^3-4*b^2*c^3*e+b^2*c^2*d^2;
           
p_term = (8*a*c-3*b^2)/(8*a^2);
q_term = (b^3-4*a*b*c+8*a^2*d)/(8*a^3);

delta0_term = c^2-3*b*d+12*a*e;
delta1_term = 2*c^3-9*b*c*d+27*b^2*e+27*a*d^2-72*a*c*e;

Q_term = ((delta1_term+sqrt(-27*discriminant))/2)^(1/3);
S_term = 0.5*sqrt(-2/3*p_term+1/(3*a)*(Q_term+delta0_term/Q_term));

% We are interested in the biggest real solution to the polynomic equation

% Vmed_sols(1) = -b/(4*a)+S_term+0.5*sqrt(-4*S_term^2-2*p_term+q_term/S_term);
% Vmed_sols(2) = -b/(4*a)-S_term+0.5*sqrt(-4*S_term^2-2*p_term+q_term/S_term);
% Vmed_sols(3) = -b/(4*a)+S_term+0.5*sqrt(-4*S_term^2-2*p_term-q_term/S_term);
% Vmed_sols(4) = -b/(4*a)-S_term+0.5*sqrt(-4*S_term^2-2*p_term-q_term/S_term);

% Vmed = real(-b/(4*a)+S_term+0.5*sqrt(-4*S_term^2-2*p_term-q_term/S_term));
Vmed = real(-b/(4*a)+S_term+0.5*sqrt(-4*S_term^2-2*p_term-q_term/S_term));

% Solve the rest of variables
V_new = max(2*(Vmed-V_old)+V_old,1);
phi = acos(1/sqrt(V_new^2*latcurv/g+1));
Cl = m*g/(0.5*rho*S*V_new^2*cos(phi));
Cd = Cd0 + K*Cl^2;
L = 0.5*rho*V_new^2*S*Cl;
D = 0.5*rho*V_new^2*S*Cd;
alpha = Cl/Clalpha; % Cl0 considered zero
lat_G = (V_new^2*latcurv)/9.8056;