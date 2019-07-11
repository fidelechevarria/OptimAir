function [L,D] = dyn_forces(Clalpha,rho,S,Cd0,Cl0,Cdp,K,alpha,V,p)
Cl = Clalpha.*alpha + Cl0;
L = 0.5.*rho.*V.^2.*S.*Cl;
D = 0.5.*rho.*V.^2.*S.*(Cd0+K.*Cl.^2+Cdp*abs(p));
end