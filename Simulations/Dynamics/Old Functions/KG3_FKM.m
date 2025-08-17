function [r08, R, T_all] = KG3_FKM(q, Ts)
% Creating the generic rotation matrix for each q
n = length(q);
Rs = {};
for i = 1:n
    R = [cos(q(i)), -sin(q(i)), 0, 0;
         sin(q(i)),  cos(q(i)), 0, 0;
         0,        0,       1, 0;
         0,        0,       0, 1];
    Rs{i} = R;
end

% Computing the transformation matrices
T_all = {};
T07 = eye(4);
% T_all{1} = eye(4);
for j = 1:n
    A = Ts{j} * Rs{j};
    T07 = T07*A;
    T_all{j} = T07;
end

T08 = T07*Ts{8};
T_all{8} = T08;
r08 = T08(1:end-1,end);
R = T08(1:3,1:3);
end