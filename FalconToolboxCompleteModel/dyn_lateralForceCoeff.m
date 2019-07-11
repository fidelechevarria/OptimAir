function Cy = dyn_lateralForceCoeff(Cyb,Cydr,Cyda,Cyp,Cyr,beta,dr,da,coeffA,p,r)
Cy = Cyb.*beta+Cydr.*dr+Cyda.*da+coeffA.*(Cyp.*p+Cyr.*r);
end

