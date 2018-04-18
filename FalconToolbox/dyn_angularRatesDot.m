function [p_dot,q_dot,r_dot] = dyn_angularRatesDot(Ix,Iy,Iz,Ixz,p,q,r,LL,MM,NN)
aux = Ix.*Iz-Ixz.^2;
p_dot = (Ixz.*(Ix-Iy+Iz).*p.*q-(Iz.*(Iz-Iy)+Ixz.^2).*q.*r+Iz.*LL+Ixz.*NN)./aux;
q_dot = ((Iz-Ix).*p.*r-Ixz.*(p.^2-r.^2)+MM)./Iy;
r_dot = (((Ix-Iy).*Ix+Ixz.^2).*p.*q-Ixz.*(Ix-Iy+Iz).*q.*r+Ixz.*LL+Ix.*NN)./aux;
end

