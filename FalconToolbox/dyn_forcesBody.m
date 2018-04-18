function [Xa,Ya,Za,Xt] = dyn_forcesBody(alpha,beta,D,Y,L,Tmax,dt)
ca = cos(alpha);
sa = sin(alpha);
cb = cos(beta);
sb = sin(beta);
Xa = -ca.*cb.*D-ca.*sb.*Y+sa.*L;
Ya = -sb.*D+cb.*Y;
Za = -sa.*cb.*D-sa.*sb.*Y-ca.*L;
Xt = Tmax.*dt;
end

