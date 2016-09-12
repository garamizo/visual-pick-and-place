function [qp, dev] = ik(t, q)
% Inverse Kinematics
%   t: coordinate target
%   q: initial joint angle guess
%   qp: resulting angle
%   dev: pose deviation, coordinate space

    ITER = 100;      % maximum number of iterations
    TOL = 1e-2;     % maximum error tolerance
    
    for k = 1 : ITER
        
        s = fk_fcn(q); % joint to space function
        e = t - s;          % pose error, cartesian space
        
        if norm(e) < TOL    % solution is close enough
            break
        end

        J = jacobian1(q);    % jacobian, d pose / d joint
        
        dq = J\e;
        if norm(dq) > 3
            factor = 1/norm(dq)^2;
        else
            factor = 1;
        end
        
        q = q + dq*factor;     % update joint angles

    end
%     fprintf('Solution completed: %d steps required', k)
    qp = q;
    dev = t - fk_fcn(q); % deviation from solution
end

function J = jacobian1(q)
% Jacobian Matrix
%   dS/dQ
    dof = length(q);
    EPS = 1e-3;
    J = zeros(dof, dof);
    for k = 1 : dof
        q1 = q - EPS*((1:dof).'==k);
        s1 = fk_fcn(q1);
        q2 = q + EPS*((1:dof).'==k);
        s2 = fk_fcn(q2);
        J(:,k) = (s2-s1)/(2*EPS);
    end
end