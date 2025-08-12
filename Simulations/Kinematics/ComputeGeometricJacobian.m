%%% Joshua Thomas
%%% C3376353

function J = ComputeGeometricJacobian(T_all)
% T_all: cell array of transforms T{0}, T{1}, ..., T{n}
% where T_all{end} is T0->8
% T{i} is T0->i
n = numel(T_all)-1;
T0 = T_all{1};
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
