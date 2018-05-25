function [A_norm] = dyn_accelNorm(ax,ay,az)
A_norm = norm([ax ay az]);
end