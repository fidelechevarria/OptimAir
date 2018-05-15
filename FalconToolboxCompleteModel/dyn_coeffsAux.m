function [coeffA,coeffB] = dyn_coeffsAux(b,c,V)
coeffA = (b./(2.*V));
coeffB = (c./(2.*V));
end

