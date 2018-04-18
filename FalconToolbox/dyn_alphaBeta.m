function [V,alpha,beta] = dyn_alphaBeta(vx,vy,vz)
V = sqrt(vx.^2+vy.^2+vz.^2);
alpha = atan(vz./vx);
beta = asin(vy./V);
end

