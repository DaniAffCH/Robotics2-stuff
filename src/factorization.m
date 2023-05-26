function [S, A] = factorization(M, q, t)
%   takes as inputs:
%   -M: inertia-Matrix
%   -q: vector of coordinates composed like this [q1(t),q2(t)] as a function of time
%   -t: time, symbolic variable
%   and outputs:
%   -S: factorization such that c(q,qdot)=S*qdot and Mdot-2S is skew-symmetric
%   -A: factorization such that c(q,qdot)=A*qdot and Mdot-2S is not
%   skew-symmetric, it is obtained from A = S + S0 where S0 is such that S0*qdot = 0
    size=length(q);
    S=sym(eye(size));
    c=sym(zeros(size, 1));
    qdot = diff(q, t);
    for i = 1:size
        C_i=jacobian(M(:,i),q) + transpose(jacobian(M(:,i),q));
        aux=eye(size);
        aux=sym(aux);
        for j=1:size
            aux(j,:)=jacobian(M(j,:),q(i));
        end
        C_i=C_i-aux;
        C_i=C_i/2;
        fprintf(1,'Christoffel C%d = \n',i);
        disp(C_i);
        S(i,:)=qdot*C_i;
        c(i)=S(i,:)*qdot.';
    end
    fprintf('Coriolis and centrifugal term = \n');
    disp(c);
    
    if size == 2
        S0 = [-qdot(2) qdot(1);0 0];
    elseif size == 3
        S0 = [0, -qdot(3), qdot(2); qdot(3), 0, -qdot(1);-qdot(2), qdot(1), 0];
    else 
        fprintf('Not implemented!');
    end

    A = simplify(S+S0);
    S = simplify(S);

    fprintf('Skew symmetric factorization= \n');
    disp(S);

    fprintf('Another factorization= \n');
    disp(A);

    Mdot = simplify(diff(M, t));
    fprintf('M dot = \n');
    disp(Mdot);

    error_skew = (Mdot-2*S) + (Mdot-2*S).';
    error_alternative = (Mdot-2*A) + (Mdot-2*A).';

    if isequal(error_skew, sym(zeros(size))) && isequal(simplify(S*qdot.'), c)
        disp('Good job');
    else
        disp('Error')
    end
    if ~isequal(error_alternative, sym(zeros(size))) && isequal(simplify(A*qdot.'), c)
        disp('Good job');
    else
        disp('Error');
    end
end