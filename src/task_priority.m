function [qdot] = task_priority(q,qs,p1,p2,rd1dot,rd2dot)
%   takes as inputs: 
%   -q: vector of coordinates composed like this [q1,q2,q3]
%   -qs: vector of coordinates of the actual configuration
%   [0.5,pi/2,3.8660]
%   -p1,p2: task vectors (they must be function of q!)
%   -rd1dot,rd2dot: desired tasks with rd1 HIGHEST PRIORITY TASK
%   and output:
%   qdot->desired joint velocity command

    J1=simplify(jacobian(p1,q));
    J2=simplify(jacobian(p2,q));
    fprintf("First Jacobian: ");
    disp(J1);
    fprintf("Second Jacobian: ");
    disp(J2);

    J1=subs(J1,q,qs);
    J2=subs(J2,q,qs);
    size=length(q);
    qdot=pinv(J1)*transpose(rd1dot)+pinv(simplify(J2*(eye(size)-pinv(J1)*J1)))*(rd2dot-transpose(J2*pinv(J1)*transpose(rd1dot)));
    qdot=simplify(qdot)
end

