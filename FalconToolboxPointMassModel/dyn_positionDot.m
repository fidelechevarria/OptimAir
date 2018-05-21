function [x_dot,y_dot,h_dot] = dyn_positionDot(q0,q1,q2,q3,V)
x_dot = V.*(q0.^2+q1.^2-q2.^2-q3.^2);
y_dot = 2.*V.*(q0.*q3+q1.*q2);
h_dot = 2.*V.*(q0.*q2-q1.*q3);
end