function [q0_dot,q1_dot,q2_dot,q3_dot] = dyn_quatDot(q0,q1,q2,q3,p,q,r)
q0_dot = -0.5.*(q1.*p+q2.*q+q3.*r);
q1_dot =  0.5.*(q0.*p+q2.*r-q3.*q);
q2_dot =  0.5.*(q0.*q+q3.*p-q1.*r);
q3_dot =  0.5.*(q0.*r+q1.*q-q2.*p);
end