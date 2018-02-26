function [arc] = optimFP_arclength(precision,numOfWaypoints,bx,cx,dx,by,cy,dy)

% ANALYTIC APPROACH (USING INTEGRAL EXPLICIT SOLUTION)
arc = zeros(precision*numOfWaypoints,1); % Initialize arc
int = cell(numOfWaypoints,1);
eval_int = zeros(numOfWaypoints*precision,1);
for i = 0:numOfWaypoints-1
    int{i+1} = @(u) sqrt((bx(i+1)+2.*cx(i+1).*u + 3.*dx(i+1).*u.^2).^2 + (by(i+1)+2.*cy(i+1).*u + 3.*dy(i+1).*u.^2).^2);
    for j = 1:precision
        eval_int(j+i*precision) = feval(int{i+1},j/precision);
    end
end

for i = 1:numOfWaypoints*precision % Trapezoidal integration approximation
    if i == numOfWaypoints*precision
        highest = max(eval_int(1),eval_int(i));
        lowest = min(eval_int(1),eval_int(i));
        arc(i) = 1/precision * (lowest + (highest-lowest)/2);
    else
        highest = max(eval_int(i+1),eval_int(i));
        lowest = min(eval_int(i+1),eval_int(i));
        arc(i) = 1/precision * (lowest + (highest-lowest)/2);
    end
end