function [omega_i,v_i, v_ci, Ti] = movingFrameDH(dhtable_i, qi_dot, omega_prec, v_prec, isPrismatic, rci_from_i, I, m)
%MOVINGFRAMEGENERIC This function compute the Moving Frames parameters for
%   one step
%   example:
%   [omega_prec,v_prec, v_ci, Ti] = movingFrameGeneric([0 l1 0 q1], q1dot, [0;0;0], [0;0;0], 0, [-l1+rc1;0;0], [i1xx 0 0; 0 i1yy 0; 0 0 i1zz], m1)
%   
%   - qi_dot: symbolic for qi dot
%
%   - omega_prec: omega computed in the previous iteration (AS COLUMN VECTOR). 
%   Set [0;0;0] for the first iteration
%
%   - v_prec: velocity computed in the previous iteration (AS COLUMN
%   VECTOR). Set [0;0;0] for the first iteration
%
%   - isPrismatic: 1 if the i-th joint is prismatic, 0 otherwise.
%
%   - r_from_i: the distance from i-1 joint to i joint, expressed in i-th
%   frame (AS COLUMN VECTOR). Usually the vector has only 1 component (always I would say but better be safe)
%
%   - rci_from_i: the i-th center of mass position expressed in i-th frame
%   (AS COLUMN VECTOR)
%
%   - I: inertia matrix 
%
%   - m: mass of i-th link

H = DHMatrix(dhtable_i);
R = H(1:3,1:3);
r_from_i_minus_1 = simplify(H(1:3,4));
r_from_i = simplify(R.'*r_from_i_minus_1);

omega_i = simplify(R.' * (omega_prec + (1-isPrismatic)*qi_dot*[0;0;1]));

v_i = simplify(R.' * (v_prec + isPrismatic*qi_dot*[0;0;1]) + cross(omega_i, r_from_i));

v_ci = v_i + cross(omega_i, rci_from_i);
v_ci = simplify(v_ci);

Ti = simplifyFraction(1/2*m*(v_ci.'*v_ci) + 1/2 *omega_i.'*I*omega_i);

end
