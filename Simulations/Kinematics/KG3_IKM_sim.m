%%% Joshua Thomas
%%% C3376353

clear
close all
clc

Ndof = 7;
rho = 0.1;

%% Setting up the KG3 direct kinematic model
[transform_computed, Ts, Rs, As] = KG3_FKM;

%% Setting up the robot model
% Load the KG3 robot 
gen3 = loadrobot("kinovaGen3", "DataFormat", "column");
% show(gen3);
showdetails(gen3)

%% Place the KG3 in a random configuration
q_config = randomConfiguration(gen3);
% show(gen3, q_config)

%% Find the homogenous transform
transform = getTransform(gen3, q_config, "EndEffector_Link");

% Extracting the position vector
r08_desired = transform(1:end-1,end)

% Extracting the rotation matrix 
R_desired = transform(1:3,1:3);
% eul_desired = rotm2eul(R_desired)
quat_desired = rotm2quat(R_desired);
pose_desired = [r08_desired;quat_desired.'];

%% Constructing the IKM optimisation problem
% Converting the symbolic expression for the position vector to a function
% k = matlabFunction(r08);
% k = matlabFunction(pose);

% Defining the objective function 
function f = objfun(qtilde)
    Ndof = 7;
    W = eye(Ndof); % joint weighting matrix
    f = qtilde.'*W*qtilde;
end

qtilde = optimvar('qtilde', 7);

% Setting constraints
qtilde.LowerBound = -pi;
qtilde.UpperBound = pi;
% posConstraint = k(qtilde(1),qtilde(2),qtilde(3),qtilde(4),qtilde(5),qtilde(6),qtilde(7)) == pose_desired;
posConstraint = KG3_FKM_simple(qtilde) == pose_desired;

% Setting the inital conditions
x0.qtilde = zeros(7,1);

obj = objfun(qtilde);
prob = optimproblem('Objective', obj);
prob.Constraints.constr = posConstraint;

% Solve the optimization problem
[sol, fval] = solve(prob, x0);

% Display the results
disp('Optimal solution:');
disp(sol);
disp('Objective function value at optimal solution:');
disp(fval);

q = sol.qtilde;
% r08_achieved = k(q(1),q(2),q(3),q(4),q(5),q(6))
% pose_achieved = k(q(1),q(2),q(3),q(4),q(5),q(6), q(7))
pose_achieved = KG3_FKM_simple(q)
pose_desired

poseError = pose_achieved - pose_desired

% posError = abs(r08_achieved - r08_desired)

