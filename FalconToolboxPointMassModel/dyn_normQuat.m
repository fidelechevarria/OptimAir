function [q0_norm,q1_norm,q2_norm,q3_norm] = dyn_normQuat(q0,q1,q2,q3)
q_norm = sqrt(q0.^2 + q1.^2 + q2.^2 + q3.^2);
q0_norm = q0./q_norm;
q1_norm = q1./q_norm;
q2_norm = q2./q_norm;
q3_norm = q3./q_norm;
end