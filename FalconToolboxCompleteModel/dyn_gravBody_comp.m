function [gx,gy,gz] = dyn_gravBody_comp(roll,pitch,g)
gx = g * sin(pitch);
gy = - g * sin(roll) * cos(pitch);
gz = - g * cos(roll) * cos(pitch);
end