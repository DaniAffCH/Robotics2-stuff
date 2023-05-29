function [Mp, cp, gp, up_of_u] = coordinateChange(q, p, J, M, c, g, u, up, t)
% This function compute the dynamic model terms given a transformation of
% coordinates with an associated Jacobian
% This coordinate change can be represented by a relation p = J*q, which is a linear transformation
%   example 1 - 3R spatial:   
%   coordinateChange([q1(t); q2(t); q3(t)], [p1(t); p2(t); p3(t)], [1 0 0; 0 1 0; 0 1 1], [m11 0 0; 0 m22 m23; 0 m23 m33], [c1;c2 ;c3], [g1;g2;g3], [u1; u2;u3], [up1;up2;up3], t)

%   example 2 - 2R:
%   coordinateChange([q1(t); q2(t)], [p1(t); p2(t)], [1 0;1 1], [a1+2*a2*cos(q2(t)) a3+a2*cos(q2(t));a3+a2*cos(q2(t)) a3], [-a2*sin(q2(t))*(q2dot^2+2*q1dot*q2dot);a2*sin(q2(t))*q1dot^2], [a4*cos(q1(t))+a5*cos(q1(t)+q2(t));a5*cos(q1(t)+q2(t))], [u1; u2], [up1;up2], t)
%   use as q1dot and q2dot diff(q1(t), t)
%
%   - q: symbolic vector of coordinates, function of t [q1(t); q2(t); ...]
%
%   - p: symbolic vector of new coordinates, function of t [p1(t); p2(t), ...]
%
%   - J: Linear transformation relating p and q, p = J*q 
%
%   - M: Inertia matrix function of the original generalized coordinates
%   
%   - c: Coriolis and centrifugal terms function of the original
%   generalized coordinates and qdot
%
%   - g: gravity vector, function of the original generalized coordinates
%
%   - u: symbolic input torques [u1(t); u2(t); ...]
%, 
%   - up: symbolic new torques
%
%   - t: time, symbolic variable
%   
%   outputs:
%   - Mp: Inertia matrix function of the new coordinates
%
%   - cp: Coriolis and centrifugal terms function of the new coordinates 
%
%   - gp: gravity vector, function of the new coordinates
%
%   - up: new input torques
Jdot = diff(J, t);
pdot = diff(p, t);
Mp = simplify(pinv(J).'*M*pinv(J));
cp = simplify(pinv(J).'*c-Mp*Jdot*pinv(J)*pdot);
gp = simplify(pinv(J).'*g);
up_of_u = simplify(pinv(J).'*u);
u_of_up = simplify(J'*up);
p_of_q = J*q;
q_of_p = pinv(J)*p;
fprintf('\n\nNew Coordinates function of q = \n')
disp(p_of_q)
fprintf('\n\nGeneralized Coordinates function of p = \n')
disp(q_of_p)
fprintf('\n\nNew Inertia Matrix Mp = \n')
disp(Mp)
fprintf('\n\nNew Coriolis and Centrifugal Term cp = \n')
disp(cp)
fprintf('\n\nNew Gravity Vector gp = \n')
disp(gp)
fprintf('\n\nNew Torques function of original torques = \n')
disp(up_of_u)
fprintf('\n\nOriginal Torques function of new torques = \n')
disp(u_of_up)