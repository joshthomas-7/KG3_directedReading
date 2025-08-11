%%% Joshua Thomas
%%% c3376353

clear
clc

%% Setting up the robot model
% Load the KG3 robot 
% gen3 = loadrobot("kinovaGen3", "DataFormat", "column");
gen3 = loadrobot("kinovaGen3", "DataFormat", "column", "Version", 2);
% show(gen3);
showdetails(gen3)

%% Place the KG3 in a random configuration
q_config = randomConfiguration(gen3);
% show(gen3, q_config)

%% Find the homogenous transform
% transform = getTransform(gen3, q_config, "EndEffector_Link")
transform = getTransform(gen3, q_config, "end_effector_link")

% Extracting the position vector
p_des = transform(1:end-1,end);
% Extracting the rotation matrix 
R_desired = transform(1:3,1:3);
eul_des = rotm2eul(R_desired);
% q_des = rotm2quat(R_desired);
% pose_des = [p_des;q_des.'];
pose_des = [p_des;eul_des.'];

%% Defining the trajectory
x_ref = [0.4 0.4;
         0 -0.2;
         0.4 0.6;
         pi/2 pi/2;
         0 pi/3;
         pi/2 deg2rad(115)];


%% Solving IKM as an optimisation problem

% Setting up the IKM optimisation problem
% Precomputing the forward kinematic matricies
Ts = KG3_preCompute();

% Precomputing the analytical jacobian
[~,~,~,T_all] = KG3_FKM();
% Computing the geometric jacobian
J = geometricJacobian(T_all);
% defining temporary symbolic variables for the ZYZ euler angles
syms ph th ps
T = [0 -sin(ph) cos(ph)*sin(th);
    0 cos(ph) sin(ph)*sin(th);
    1 0 cos(th)];
TAi = [eye(3) zeros(3);
    zeros(3) inv(T)];
% Computing the analytical jacobian using the geometric jacobian.
analyticalJacobian = simplify(TAi*J, 'Steps', 15);
JA = matlabFunction(analyticalJacobian);

q0 = zeros(7,1); % initial guess
W = eye(7);
rho = 0.1;
rhoVec = 2.5e1*ones(6,1);
options = optimoptions('fmincon', 'Display', 'none');
lb = -pi*ones(7,1);
ub = pi*ones(7,1);

sigma0 = zeros(7+6,1);
lb_alt = [lb;zeros(6,1)];
ub_alt = [ub;1e3*ones(6,1)];

funAlt = @(sigma) obj(sigma, W, rhoVec);


%% Simulating the trajectory Tracking

N = width(x_ref);
x_act = zeros(size(x_ref));
for i=1:N
    % Extracting the desired pose at this time step
    x_des = x_ref(:,i);
    p_des = x_des(1:3);
    eul_des = x_des(4:end);
    
    % Solving the IKM for the desired pose
    sigma_sol = fmincon(@(sigma)obj(sigma,sigma0,W,rhoVec), sigma0, [], [], [], [], lb_alt, ub_alt,...
        @(sigma) constraints(sigma, p_des, eul_des, Ts), options);

    % Extract the actual pose after solving the inverse kinematics
    q = sigma_sol(1:7);
    [p_act, R_act] = KG3_FKM_simple(q, Ts);
    eul_act = rotm2eul(R_act);
    x_act(:,i) = [p_act; eul_act.']; 

    % Using the previous joint configuration as the inital guess in the
    % next IKM solve
    sigma0 = [q;zeros(6,1)];
    

end


function cost = obj(sigma, sigma0, W, rhoVec)
    
    q = sigma(1:7);
    s = sigma(8:end);
    q0 = sigma0(1:7);

    cost = (q-q0).'*W*(q-q0) + rhoVec.'*s;
end

function [c, ceq] = constraints(sigma, p_des, eul_des, Ts)
    q = sigma(1:7);
    s = sigma(8:end);
    [p, R] = KG3_FKM_simple(q, Ts);
    eul = rotm2eul(R);
    % qu = rotm2quat(R);
    
    pose = [p;eul.'];
    % pose = [p;qu.'];
    % pose_des = [p_des;q_des.'];
    pose_des = [p_des;eul_des];

    c_upper = pose - s - pose_des;
    c_lower = -pose -s + pose_des;

    c = [c_upper;c_lower];
    ceq = [];
end




% sigma_sol = fmincon(funAlt, sigma0, [], [], [], [], lb_alt, ub_alt,...
%     @(sigma) constraints(sigma, p_des, eul_des, Ts), options);

% q = sigma_sol(1:7);
% transform_act = getTransform(gen3, q, "end_effector_link")
% % Extract the actual pose after solving the inverse kinematics
% [p_act, R_act] = KG3_FKM_simple(q, Ts);
% % qu_act = rotm2quat(R_act);
% eul_act = rotm2eul(R_act);
% 
% pose_des
% % pose_act = [p_act; qu_act.']
% pose_act = [p_act; eul_act.']
% 
% pose_err = pose_des - pose_act


%% Plotting
figure(1)
subplot(2,1,1)
hold on
plot(x_ref(1,:), '--r', 'LineWidth',3)
plot(x_act(1,:), 'ro', 'LineWidth', 3)
plot(x_ref(2,:), '--b', 'LineWidth', 3)
plot(x_act(2,:), 'bo', 'LineWidth', 3)
plot(x_ref(3,:), '--g', 'LineWidth', 3)
plot(x_act(3,:), 'go', 'LineWidth', 3)

ylabel('Position (m)', 'FontSize', 18)
xlabel('Sample Point', 'FontSize', 18)
legend('x*', 'x', 'y*', 'y', 'z*', 'z')

subplot(2,1,2)
hold on
plot(x_ref(4,:), '--r', 'LineWidth',3)
plot(x_act(4,:), 'ro', 'LineWidth', 3)
plot(x_ref(5,:), '--b', 'LineWidth', 3)
plot(x_act(5,:), 'bo', 'LineWidth', 3)
plot(x_ref(6,:), '--g', 'LineWidth', 3)
plot(x_act(6,:), 'go', 'LineWidth', 3)

ylabel('Orientation (^\circ)', 'FontSize', 18)
xlabel('Sample Point', 'FontSize', 18)
legend('x*', 'x', 'y*', 'y', 'z*', 'z')