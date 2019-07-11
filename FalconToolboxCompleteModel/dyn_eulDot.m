function [roll_dot,pitch_dot,yaw_dot] = dyn_eulDot(p,q,r,tp,sr,cr,cp)
roll_dot = p+tp.*(q.*sr+r.*cr);
pitch_dot = q.*cr-r.*sr;
yaw_dot = (q.*sr+r.*cr)./cp;
end

