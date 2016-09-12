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
%     r1 = -atan2(T(1,2), T(1,1));  % don't use atan2
    r3 = atan2(T(3,2), T(3,3));
    r2 = atan2(-T(3,1), (T(3,3)/cos(r3)));
    s = [x; y; z; r2];
end

function A = DH(angle, offset, twist, length)
% Denavit Hartemberg transformation
    A = [Rz(angle) [0; 0; offset]; 0 0 0 1] * [Rx(twist) [length; 0; 0]; 0 0 0 1];
end


