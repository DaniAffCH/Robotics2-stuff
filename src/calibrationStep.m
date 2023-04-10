function [calibOffset] = calibrationStep(f, r, alpha, a, d, theta, active)
% CALIBRATION_STEP computes ONE STEP of calibration algorithm
% 
% PARAMETERS: 
%   - f: is the direct kinematics (i.e. for a planar 2R
%        f=[l1*cos(q1)+l2*cos(q1+q2);l1*sin(q1)+l2*sin(q1+q2)]).
%   - r: is the respective task measured coordinate (i.e. r=[2,3])
%   - alpha: 2 rows matrix for alpha parameters; first row are the symbolic
%            and second rows the values (i.e. alpha=[alpha1,alpha2;3,4]).
%   - a, d, theta: same as alpha.
%   - active: vector of 0 and 1 indicating which parameters have to be
%             adjusted (i.e. active=[alpha,a,d,theta]=[0,1,0,0] will only
%             adjust the 'a' parameters of DH table).
% 
% USAGE EXAMPLE: 
%   calibration_step(r, [1.6925,0.7425],[],[l1,l2;1,1],[],[q1,q2;pi/4,-pi/4],[0,1,0,0])



[alphaParameters, alphaNominals] = deal([],[]);
if not(isempty(alpha)) [alphaParameters, alphaNominals] = deal(alpha(1,:), alpha(2,:)); end

[aParameters, aNominals] = deal([],[]);
if not(isempty(a)) [aParameters, aNominals] = deal(a(1,:), a(2,:)); end

[dParameters, dNominals] = deal([],[]);
if not(isempty(d)) [dParameters, dNominals] = deal(d(1,:), d(2,:)); end

[thetaParameters, thetaNominals] = deal([],[]);
if not(isempty(theta)) [thetaParameters, thetaNominals] = deal(theta(1,:), theta(2,:)); end

regressor = sym([]);
currentColTracker = 0;

dofs = -1;

alphaMatch = length(alphaParameters) == length(alphaNominals);
dMatch = length(dParameters) == length(dNominals);
aMatch = length(aParameters) == length(aNominals);
thetaMatch = length(thetaParameters) == length(thetaNominals);

rNominal = f;

if(not(isempty(alphaParameters)) && not(isempty(alphaNominals)) && alphaMatch && (dofs == -1 || length(alphaParameters) == dofs) && active(1))
    disp("========Applying alpha gradient========");
    dofs = length(alphaParameters);
    diffsym = simplify(jacobian(f, alphaParameters));
    disp("symbolic jacobian:")
    disp(diffsym);
    diff = subs(diffsym, alphaParameters, alphaNominals);
    regressor(:, currentColTracker+1:currentColTracker+length(alphaParameters)) = diff;
    disp("partial regressor:")
    disp(regressor);
    currentColTracker = currentColTracker+length(alphaParameters);
    disp("========End alpha========");
else
    disp("Alpha parameters/nominal values empty or ill-conditioned. Skipped")
end

if(not(isempty(aParameters)) && not(isempty(aNominals)) && aMatch && (dofs == -1 || length(aParameters) == dofs) && active(2))
    disp("========Applying a gradient========");
    dofs = length(aParameters);
    diffsym = simplify(jacobian(f, aParameters));
    disp("symbolic jacobian:")
    disp(diffsym);
    diff = subs(diffsym, aParameters, aNominals);
    regressor(:, currentColTracker+1:currentColTracker+length(aParameters)) = diff;
    disp("partial regressor:")
    disp(regressor);
    currentColTracker = currentColTracker+length(aParameters);
    disp("========End a========");
else
    disp("a parameters/nominal values empty or ill-conditioned. Skipped")
end

if(not(isempty(dParameters)) && not(isempty(dNominals)) && dMatch && (dofs == -1 || length(dParameters) == dofs) && active(3))
    disp("========Applying d gradient========");
    dofs = length(dParameters);
    diffsym = simplify(jacobian(f, dParameters));
    disp("symbolic jacobian:")
    disp(diffsym);
    diff = subs(diffsym, dParameters, dNominals);
    regressor(:, currentColTracker+1:currentColTracker+length(cParameters)) = diff;
    disp("partial regressor:")
    disp(regressor);
    currentColTracker = currentColTracker+length(dParameters);
    disp("========End d========");
else
    disp("d parameters/nominal values empty or ill-conditioned. Skipped")
end


if(not(isempty(thetaParameters)) && not(isempty(thetaNominals)) && thetaMatch && (dofs == -1 || length(thetaParameters) == dofs) && active(4))
    disp("========Applying theta gradient========");
    dofs = length(thetaParameters);
    diffsym = simplify(jacobian(f, thetaParameters));
    disp("symbolic jacobian:")
    disp(diffsym);
    diff = subs(diffsym, thetaParameters, thetaNominals);
    regressor(:, currentColTracker+1:currentColTracker+length(thetaParameters)) = diff;
    disp("partial regressor:")
    disp(regressor);
    currentColTracker = currentColTracker+length(thetaParameters);
    disp("========End theta========");
else
    disp("Theta parameters/nominal values empty or ill-conditioned. Skipped")
end

if(alphaMatch)
    regressor = subs(regressor, alphaParameters, alphaNominals);
    rNominal = subs(rNominal, alphaParameters, alphaNominals);
end

if(dMatch)
    regressor = subs(regressor, dParameters, dNominals);
    rNominal = subs(rNominal, dParameters, dNominals);
end

if(aMatch)
    regressor = subs(regressor, aParameters, aNominals);
    rNominal = subs(rNominal, aParameters, aNominals);
end

if(thetaMatch)
    regressor = subs(regressor, thetaParameters, thetaNominals);
    rNominal = subs(rNominal, thetaParameters, thetaNominals);
end

regressor = double(regressor);
rNominal = double(rNominal);

disp("Final regressor: ");
disp(regressor);

if(length(r) ~= size(regressor,1))
    fprintf("observation's size mismatch, expected:%d given:%d\n",size(regressor,1), length(r));
    return;
end

deltar = r.' - rNominal;

calibOffset = pinv(regressor)*deltar;
end
