function [r08, R] = KG3_FKM_simple(q, Ts)
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

% % Creating the coordinate transformations
% Ts = {};
% 
% T01 = [1 0 0 0;
%        0 -1 0 0;
%        0 0 -1 0.1564;
%        0 0 0 1];
% Ts{1} = T01;
% 
% T12 = [1 0 0 0;
%        0 0 -1 0.0054;
%        0 1 0 -0.1284;
%        0 0 0 1];
% Ts{2} = T12;
% 
% T23 = [1 0 0 0;
%     0 0 1 -0.2104;
%     0 -1 0 -0.00640;
%     0 0 0 1];
% Ts{3} = T23;
% 
% T34 = [1 0 0 0;
%     0 0 -1 0.0064;
%     0 1 0 -0.2104;
%     0 0 0 1];
% Ts{4} = T34;
% 
% T45 = [1 0 0 0;
%     0 0 1 -0.2084;
%     0 -1 0 -0.0064;
%     0 0 0 1];
% Ts{5} = T45;
% 
% T56 = [1 0 0 0;
%     0 0 -1 0;
%     0 1 0 -0.1059;
%     0 0 0 1];
% Ts{6} = T56;
% 
% T67 = [1 0 0 0;
%     0 0 1 -0.1059;
%     0 -1 0 0;
%     0 0 0 1];
% Ts{7} = T67;
% 
% T78 = [1 0 0 0;
%        0 -1 0 0;
%        0 0 -1 -0.0615;
%        0 0 0 1];

% Computing the transformation matrices
T07 = eye(4);
for j = 1:n
    A = Ts{j} * Rs{j};
    T07 = T07*A;
end

T08 = T07*Ts{8};
r08 = T08(1:end-1,end);
R = T08(1:3,1:3);
end