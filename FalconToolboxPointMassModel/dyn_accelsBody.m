function [Ax,Ay,Az] = dyn_accelsBody(gx,gy,gz,q,r,V,V_dot)
Ax = gx + V_dot;
Ay = gy + V * r;
Az = gz - V * q;
end