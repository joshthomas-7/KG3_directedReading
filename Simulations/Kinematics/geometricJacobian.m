%%% Joshua Thomas
%%% C3376353

function J = geometricJacobian(T_all)
% T_all: cell array of transforms T{0}, T{1}, ..., T{n}
% where T_all{end} is T0->ee
% For n joints, returns 6 x n Jacobian
n = numel(T_all)-1;
T0 = T_all{1}; % should be identity
p_ee = T_all{end}(1:3,4);
J = zeros(6,n);

for i = 1:n
    Ti = T_all{i}; % T0->i-1
    z = Ti(1:3,3);      % z_{i-1} axis (assuming z is joint axis)
    p = Ti(1:3,4);      % origin of frame i-1
    Jv = cross(z, p_ee - p);
    Jw = z;
    J(:,i) = [Jv; Jw];
end
end
