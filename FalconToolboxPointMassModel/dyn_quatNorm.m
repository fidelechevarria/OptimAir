function [quat_norm] = dyn_quatNorm(q0,q1,q2,q3)
quat_norm = sqrt(q0.^2 + q1.^2 + q2.^2 + q3.^2);
end