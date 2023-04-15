function [com] = PlanarNR_COMinZero(rci_from_i, linkLengths, qSym)
%COMINZERO It computes the i-th COM position expressed in 0 frame for planar
%N-R robots
%   - rci_from_i: column vector, position of COM expressed in frame i
%   - linkLengths: row vector, the link lengths up to the i-th link
%   - qSym: row vector, the q symbolics up to the i-th link
%
% Usage example for COM of 2-nd link:
% PlanarNR_COMinZero([rc2x;rc2y;0], [l1, l2], [q1, q2])

if(length(linkLengths) ~= length(qSym))
    disp("linkLengths dimension doesn't match qSym dimension");
    return;
end

addpath(genpath("../externals/Dynamics"));

T = eye(4);

for i= 1 : length(linkLengths)
    T = T * dh_step([0, 0, linkLengths(i), qSym(i)]);
end

tipPos = T * [0;0;0;1];
tipPos = tipPos(1:3, :);

rot = T(1:3, 1:3);

com = simplify(tipPos + rot*rci_from_i);

end

