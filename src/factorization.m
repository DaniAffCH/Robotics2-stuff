function [S] = factorization(M,q,qdot)
%   takes as inputs:
%   -M: inertia-Matrix
%   -q: vector of coordinates composed like this [q1,q2]
%   -qdot: vector of derivates composed like this [q1dot,q2dot]
%   and outputs:
%   -S: factorization such that c(q,qdot)=S*qdot and Mdot-2S is skew-symmetric

    size=length(q);
    S=eye(size)
    S=sym(S)
    for i = 1:size
        C=jacobian(M(:,i),q)+transpose(jacobian(M(:,i),q));
        aux=eye(size)
        aux=sym(aux)
        for j=1:size
            aux(j,:)=jacobian(M(j,:),q(i))
        end;
        C=C-aux;
        C=C/2;
        disp("Christoffel symbol: ");
        disp(C);
        S(i,:)=qdot*C;
    end;