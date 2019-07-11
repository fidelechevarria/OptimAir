function Cll = dyn_rollingMomentCoeff(Cllb,Cllda,Clldr,Cllp,Cllr,beta,da,dr,coeffA,p,r)
Cll = Cllb.*beta+Cllda.*da+Clldr.*dr+coeffA.*(Cllp.*p+Cllr.*r);
end

