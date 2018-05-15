function [vx_dot,vy_dot,vz_dot] = dyn_velBodyDot(vx,vy,vz,p,q,r,Xa,Xt,Ya,Za,g,m,sp,cp,sr,cr)
vx_dot = r.*vy-q.*vz-g.*sp+(Xa+Xt)./m;
vy_dot = -r.*vx+p.*vz+g.*sr.*cp+Ya./m;
vz_dot = q.*vx-p.*vy+g.*cr.*cp+Za./m;
end

