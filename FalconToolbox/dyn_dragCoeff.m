function Cd = dyn_dragCoeff(Cd0,Cda2,Cdb,alpha,beta)
Cd = Cd0+Cda2.*alpha+Cdb.*abs(beta);
end

