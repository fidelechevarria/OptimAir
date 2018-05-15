function Cnn = dyn_yawingMomentCoeff(Cnnb,Cnnda,Cnndr,Cnnp,Cnnr,beta,da,dr,coeffA,p,r)
Cnn = Cnnb.*beta+Cnnda.*da+Cnndr.*dr+coeffA.*(Cnnp.*p+Cnnr.*r);
end

