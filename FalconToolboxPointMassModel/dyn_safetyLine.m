function [SL_A,SL_B] = dyn_safetyLine(x,y,SL_A_m,SL_A_n,SL_B_m,SL_B_n)
SL_A = ((x-SL_A_n)/SL_A_m)-y;
SL_B = ((x-SL_B_n)/SL_B_m)-y;
end