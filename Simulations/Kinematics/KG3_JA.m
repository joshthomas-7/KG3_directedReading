%%% C3376353
%%% Joshua Thomas

function [JA, T] = KG3_JA(phi, theta, J)
% Computes the analytical Jacobian using the geometric Jacobian

T = [0 -sin(phi) cos(phi)*sin(theta);
    0 cos(phi) sin(phi)*sin(theta);
    1 0 cos(theta)];

% Ti = [  -(cos(phi)*cos(theta))/(sin(theta)*cos(phi)^2 + sin(theta)*sin(phi)^2) -(cos(theta)*sin(phi))/(sin(theta)*cos(phi)^2 + sin(theta)*sin(phi)^2) 1;
%         -sin(phi)/(cos(phi)^2 + sin(phi)^2) cos(phi)/(cos(phi)^2 + sin(phi)^2) 0;
%         cos(phi)/(sin(theta)*cos(phi)^2 + sin(theta)*sin(phi)^2) sin(phi)/(sin(theta)*cos(phi)^2 + sin(theta)*sin(phi)^2) 0];


TAi = [eye(3) zeros(3);
    zeros(3) inv(T)];

JA = TAi*J;



end