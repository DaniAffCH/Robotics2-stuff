function [out] = wpinv(J,W)
%WPINV Weighted PseudoInverse
out = (W\J.')/(J/W*J.');
end

