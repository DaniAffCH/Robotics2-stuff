function [omega_i,v_i, v_ci, Ti] = movingFrameGeneric(alpha_i,theta_i, qi_dot, omega_prec, v_prec, isPrismatic, r_from_i, rci_from_i, I, m)
%MOVINGFRAMEGENERIC This function compute the Moving Frames parameters for
%   one step
%   example:  
%   [omega_i v_i v_ci Ti] = movingFrameGeneric(0, q1, q1dot, [0;0;0], [0;0;0], 0, [l1;0;0], [-l1+rc1;0;0], [i1xx 0 0; 0 i1yy 0; 0 0 i1zz], m1)
%
%   - alpha_i: DH parameter associated to i-th link. Can be either symbolic
%   or numeric (remember, alpha_i is the twist angle between z_i-1 and z_i)
%
%   - theta_i: DH parameter associated to i-th link. Can be either symbolic
%   or numeric
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

R = [cos(theta_i)   -cos(alpha_i)*sin(theta_i)  sin(alpha_i)*sin(theta_i); 
     sin(theta_i)   cos(alpha_i)*cos(theta_i)   -sin(alpha_i)*cos(theta_i);
     0              sin(alpha_i)                cos(alpha_i)];

disp(R);

omega_i = R.' * (omega_prec + (1-isPrismatic)*qi_dot*[0;0;1]);
omega_i = simplify(omega_i);

v_i = R.' * (v_prec + isPrismatic*qi_dot*[0;0;1]) + cross(omega_i, r_from_i);
v_i = simplify(v_i);

v_ci = v_i + cross(omega_i, rci_from_i);
v_ci = simplify(v_ci);

Ti = 1/2*m*(v_ci.'*v_ci) + 1/2 *omega_i.'*I*omega_i;

end
