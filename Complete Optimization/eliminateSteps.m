function [ output ] = eliminateSteps( input )
    output = zeros(size(input));
    jump_raw_reference = 0;
    slope_raw_reference = 0;
    for i = 1:numel(input)-1
        if ((input(i) - input(i+1)) >= 1.5) % Negative slope
            jump_raw_reference = [jump_raw_reference i];
            slope_raw_reference = [slope_raw_reference -1];
        elseif ((input(i) - input(i+1)) <= -1.5) % Positive slope
            jump_raw_reference = [jump_raw_reference i];
            slope_raw_reference = [slope_raw_reference 1];
        end
    end
    jump_reference = jump_raw_reference(2:end);
    slope_reference = slope_raw_reference(2:end);
    for i = 1:numel(jump_reference)
        for j = 1:numel(input)
            if (j > jump_reference(i))
                input(j) = input(j) - 2*pi*slope_reference(i);
            end
        end
    end
output = input;
end

