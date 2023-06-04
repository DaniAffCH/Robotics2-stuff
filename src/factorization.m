function [S, c, A] = factorization(M, q, t)
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
        c(i)=simplifyFraction(S(i,:)*qdot.');
    end
    fprintf('Coriolis and centrifugal term = \n');
    disp(c);
    
    S = simplifyFraction(S);
    fprintf('Skew symmetric factorization= \n');
    disp(S);

    Mdot = simplifyFraction(diff(M, t));
    fprintf('M dot = \n');
    disp(Mdot);
    
    error_skew = simplifyFraction((Mdot-2*S) + (Mdot-2*S).');
    
    if size>3
        fprintf('S0, Not implemented! Only one factorization available');
        A = eye(size);
    end

    if size == 2
        S0 = [-qdot(2) qdot(1);0 0];
        A = simplifyFraction(S+S0);
        fprintf('Another factorization= \n');
        disp(A);
        error_alternative = simplifyFraction((Mdot-2*A) + (Mdot-2*A).');
    elseif size == 3
        S0 = [0, -qdot(3), qdot(2); qdot(3), 0, -qdot(1);-qdot(2), qdot(1), 0];
        A = simplifyFraction(S+S0);
        fprintf('Another factorization= \n');
        disp(A);
        error_alternative = simplify((Mdot-2*A) + (Mdot-2*A).');
    end

    disp('Mdot-2S:')
    disp(simplify(Mdot-2*S))

    if isequal(error_skew, sym(zeros(size))) && isequal(simplifyFraction(S*qdot.'), c)
        disp('Good job');
    else
        disp('Error');    
    end
    if size<=3
        if ~isequal(error_alternative, sym(zeros(size))) && isequal(simplifyFraction(A*qdot.'), c)
            disp('Good job');
        else
            disp('Error');
        end
    end
end