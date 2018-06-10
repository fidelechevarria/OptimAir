function [SL] = dyn_safetyLine(x,y,SL_m,SL_n)
SL = ((x-SL_n)/SL_m)-y;
end