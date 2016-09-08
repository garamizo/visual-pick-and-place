function s = fk_fcn(q)
% Forward kinematics
%   Transformation matrix for the end-effector
%   For AL5D robot
    T01 = DH(q(1), .105, -pi/2, .02);
    T12 = DH(q(2)-pi/2, 0, 0, 0.145);
    T23 = DH(q(3)+pi/2, 0, 0, 0.185);
    T34 = DH(q(4), 0, pi/2, .077);
    T = T01 * T12 * T23 * T34;
    
    x = T(1,4);
    y = T(2,4);
    z = T(3,4);
%     r1 = -atan(T(1,2)/T(1,1));  % don't use atan2
    r2 = asin(-T(3,1)); % pitch (ZYX)
    s = [x; y; z; r2];
end

function A = DH(angle, offset, twist, length)
% Denavit Hartemberg transformation
    A = [Rz(angle) [0; 0; offset]; 0 0 0 1] * [Rx(twist) [length; 0; 0]; 0 0 0 1];
end

function R = Rx(q)
    R = [1 0 0; 0 cos(q) -sin(q); 0 sin(q) cos(q)];
end

function R = Ry(q)
    R = [cos(q) 0 sin(q); 0 1 0; -sin(q) 0 cos(q)];
end

function R = Rz(q)
    R = [cos(q) -sin(q) 0; sin(q) cos(q) 0; 0 0 1];
end