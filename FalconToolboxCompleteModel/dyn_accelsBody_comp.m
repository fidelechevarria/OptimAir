function [Ax,Ay,Az] = dyn_accelsBody_comp(gx,gy,gz,p,q,r,vx,vy,vz,vx_dot,vy_dot,vz_dot)
Ax = gx + vx_dot + (vz * q - vy * r);
Ay = gy + vy_dot + (vx * r - vz * p);
Az = gz + vz_dot + (vy * p - vx * q);
end