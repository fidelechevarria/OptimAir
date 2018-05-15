function Cmm = dyn_pitchingMomentCoeff(Cmm0,Cmma,Cmmde,Cmmdr,Cmmda,Cmmq,alpha,de,dr,da,coeffB,q)
Cmm = Cmm0+Cmma.*alpha+Cmmde.*de+Cmmdr.*dr+Cmmda.*abs(da)+coeffB.*(Cmmq.*q);
end

