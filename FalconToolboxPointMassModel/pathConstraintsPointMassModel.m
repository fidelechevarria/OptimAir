function [constraints] = pathConstraintsPointMassModel(states, ~)
% constraint interface created by falcon.m

% Extract states
q0 = states(2);
q1 = states(3);
q2 = states(4);
q3 = states(5);

% Constraint functions
quat_const = sqrt(q0.^2+q1.^2+q2.^2+q3.^2)-1;
constraints = quat_const;

end