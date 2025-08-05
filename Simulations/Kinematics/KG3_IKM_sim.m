%%% Joshua Thomas
%%% c3376353

clear
clc

%% Setting up the robot model
% Load the KG3 robot 
gen3 = loadrobot("kinovaGen3", "DataFormat", "column");
% show(gen3);
showdetails(gen3)

%% Place the KG3 in a random configuration
q_config = randomConfiguration(gen3);
% show(gen3, q_config)

%% Find the homogenous transform
transform = getTransform(gen3, q_config, "EndEffector_Link")

% Extracting the position vector
p_des = transform(1:end-1,end);
% Extracting the rotation matrix 
R_desired = transform(1:3,1:3);
q_des = rotm2quat(R_desired);
pose_des = [p_des;q_des.'];


%% Solving IKM as an optimisation problem

Ts = KG3_preCompute();
% Objective function
% fun = @(q) objectiveFunctionQuat(q, p_des, q_des, Ts);


q0 = zeros(7,1); % initial guess
W = eye(7);
rho = 0.1;
rhoVec = 2.5e1*ones(7,1);
options = optimoptions('fmincon', 'Display', 'none');
lb = -pi*ones(7,1);
ub = pi*ones(7,1);
% q_sol = fmincon(fun, q0, [], [], [], [], lb, ub, [], options);

funAlt = @(sigma) obj(sigma, W, rhoVec);

sigma0 = zeros(2*7,1);
lb_alt = [lb;zeros(7,1)];
ub_alt = [ub;1e3*ones(7,1)];
sigma_sol = fmincon(funAlt, sigma0, [], [], [], [], lb_alt, ub_alt,...
    @(sigma) constraints(sigma, p_des, q_des, Ts), options);

% function cost = objectiveFunctionQuat(q, p_des, qu_des, Ts)
%     % Forward kinematics
%     [p, R] = KG3_FKM_simple(q, Ts);
% 
%     % Convert rotation to quaternion
%     qu = rotm2quat(R);
% 
%     % Position error
%     pos_error = norm(p - p_des)^2;
% 
%     % Orientation error (dot product metric)
%     ori_error = 1 - abs(dot(qu_des, qu));
% 
%     % Weighted cost
%     cost = pos_error + 0.5*ori_error;
% end


function cost = obj(sigma, W, rhoVec)
    
    q = sigma(1:7);
    s = sigma(8:end);
    cost = q.'*W*q + rhoVec.'*s;
end

function [c, ceq] = constraints(sigma, p_des, q_des, Ts)
    q = sigma(1:7);
    s = sigma(8:end);
    [p, R] = KG3_FKM_simple(q, Ts);
    qu = rotm2quat(R);

    pose = [p;qu.'];
    pose_des = [p_des;q_des.'];

    c_upper = pose - s - pose_des;
    c_lower = -pose -s + pose_des;

    c = [c_upper;c_lower];
    ceq = [];
end




q = sigma_sol(1:7);
transform_act = getTransform(gen3, q, "EndEffector_Link")
% Extract the actual pose after solving the inverse kinematics
[p_act, R_act] = KG3_FKM_simple(q, Ts);
qu_act = rotm2quat(R_act);

pose_des
pose_act = [p_act; qu_act.']

pose_err = pose_des - pose_act