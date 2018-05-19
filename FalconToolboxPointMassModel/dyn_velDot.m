function [V_dot] = dyn_velDot(m,g,T,alpha,D,q0,q1,q2,q3)
V_dot = (T.*cos(alpha)-D+2.*m.*g.*(q1.*q3-q0.*q2))./m;
end