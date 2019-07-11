function [cr,sr,cp,sp,tp,cy,sy] = dyn_eulTrigon(roll,pitch,yaw)
cr = cos(roll);
sr = sin(roll);
cp = cos(pitch);
sp = sin(pitch);
tp = tan(pitch);
cy = cos(yaw);
sy = sin(yaw);
end

