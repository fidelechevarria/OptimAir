
function [c, ceq] = pathConstraint(x,u)
% [c, ceq] = pathConstraint(z)
%
% Computes the path constraint
  
c = [];
ceq = [sqrt(x(4,:).^2+x(5,:).^2+x(6,:).^2+x(7,:).^2)-1]; % Quaternion norm is 1

end

