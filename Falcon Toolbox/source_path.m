function [constraints] = source_path(states, ~)
% constraint interface created by falcon.m

% Extract states
q0 = states(4);
q1 = states(5);
q2 = states(6);
q3 = states(7);

% Constraint functions
quat_const = sqrt(q0.^2+q1.^2+q2.^2+q3.^2)-1;
constraints = quat_const;

end