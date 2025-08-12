%%% C3376353
%%% Joshua Thomas

function [JA, T] = KG3_analyticalJacobian(phi, theta, J)
% Computes the analytical Jacobian using the geometric Jacobian

T = [0 -sin(phi) cos(phi)*sin(theta);
    0 cos(phi) sin(phi)*sin(theta);
    1 0 cos(theta)];
TAi = [eye(3) zeros(3);
    zeros(3) inv(T)];

JA = TAi*J;



end