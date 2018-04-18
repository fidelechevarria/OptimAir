function [x_dot,y_dot,h_dot] = dyn_posDot(vx,vy,vz,sr,cr,sp,cp,sy,cy)
x_dot = vx.*cp.*cy+vy.*(-cr.*sy+sr.*sp.*cy)+vz.*(sr.*sy+cr.*sp.*cy);
y_dot = vx.*cp.*sy+vy.*(cr.*cy+sr.*sp.*sy)+vz.*(-sr.*cy+cr.*sp.*sy);
h_dot = vx.*sp-vy.*sr.*cp-vz.*cr.*cp;
end

