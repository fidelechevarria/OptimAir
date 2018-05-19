function [q,r] = dyn_angVels(m,g,T,V,q0,q1,q2,q3,L,alpha)
q = (T.*sin(alpha)+L-m.*g.*(q0.^2-q1.^2-q2.^2+q3.^2))./(m.*V);
r = (2.*m.*g.*(q0.*q1+q2.*q3))./(m.*V);
end