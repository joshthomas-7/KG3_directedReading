%%% Joshua Thomas
%%% c3376353

clear
close all
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
transform = getTransform(gen3, q_config, "end_effector_link");

% Extracting the position vector
p_des = transform(1:end-1,end);
% Extracting the rotation matrix 
R_desired = transform(1:3,1:3);
eul_des = rotm2eul(R_desired);
% q_des = rotm2quat(R_desired);
% pose_des = [p_des;q_des.'];
pose_des = [p_des;eul_des.'];

%% Defining the trajectory
% x_ref = [0.4 0.4;
%          0 -0.2;
%          0.4 0.6;
%          pi/2 pi/2;
%          0 pi/3;
%          pi/2 deg2rad(115)];

% nWaypoints = 3;
% t = zeros(1, nWaypoints);
% X = zeros(6, 3, nWaypoints);
% 
% % Waypoint 1
% t(1) = 0.0;
% X(:, :, 1) = [
%                 0.4, 0, 0;          % x position, velocity, acceleration
%                 0.0, 0, 0;          % y position, velocity, acceleration
%                 0.4, 0, 0;          % z position, velocity, acceleration
%                 pi/2, 0, 0;         % phi position, velocity, acceleration
%                 0, 0, 0;            % theta position, velocity, acceleration
%                 pi/2, 0, 0;
%               ];
% 
% 
% % Waypoint 2 accelerating towards the first resting position
% t(2) = 2;
% X(:, :, 2) = [
%                 0.4, 0, 0;          % x position, velocity, acceleration
%                 -0.2, 0.1, 0;          % y position, velocity, acceleration
%                 0.45, 0.1, 0;          % z position, velocity, acceleration
%                 pi/2, 0, 0;         % phi position, velocity, acceleration
%                 0, 0, 0;            % theta position, velocity, acceleration
%                 pi/2, 0, 0;
%               ];
% 
% % Waypoint 3 stopping at the first resting position
% t(3) = 4;
% X(:, :, 3) = [
%                 0.4, 0, 0;          % x position, velocity, acceleration
%                 -0.2, 0, 0;          % y position, velocity, acceleration
%                 0.6, 0, 0;          % z position, velocity, acceleration
%                 pi/2, 0, 0;         % phi position, velocity, acceleration
%                 0, 0, 0;            % theta position, velocity, acceleration
%                 pi/2, 0, 0;
%               ];


timePoints = [0 2 4 6 8 10];
waypoints = [0.45  0.00 0.45 pi/2 0 pi/2; 
             0.45 -0.10 0.50 pi/2 deg2rad(20) deg2rad(100); 
             0.45  -0.20 0.65 pi/2 deg2rad(45) deg2rad(110);
             0.60  -0.10 0.50 pi/2 deg2rad(20) deg2rad(100);
             0.70  0.00 0.45 pi/2 0 pi/2;
             0.70  0.00 0.45 pi/2 0 pi/2]';

% Desired velocities at waypoints
% velocities = zeros(size(waypoints)); 
velocities = [0.00 -0.01 0.01 0.00 0 0; 
              0.00 -0.01 0.01 0.00 deg2rad(10) deg2rad(5); 
              0.02  0.00 0.00 0.00 deg2rad(0) deg2rad(0);
              0.02  0.01 -0.01 0.00 deg2rad(-10) deg2rad(-5);
              0.00  0.00 0.00 0.00 0 0.00;
              0.00  0.00 0.00 0.00 0 0.00]';
ts = linspace(0,10,25);

% Compute trajectory
[x_ref, dx_ref, qdds] = cubicpolytraj(waypoints, timePoints, ts, ...
    'VelocityBoundaryCondition', velocities);

% Plot to check
figure(1);
subplot(2,1,1)
plot(ts, x_ref(1,:), 'r', ts, x_ref(2,:), 'g', ts, x_ref(3,:), 'b', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Position (m)'); title('Cartesian trajectory');
legend('x','y','z'); grid on;

subplot(2,1,2)
plot(ts, rad2deg(x_ref(4,:)), 'r', ts,rad2deg(x_ref(5,:)), 'g', ts, rad2deg(x_ref(6,:)), 'b', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Orientation (deg)');
legend('\phi','\theta','\psi'); grid on;





%% Solving IKM as an optimisation problem

% Setting up the IKM optimisation problem
% Precomputing the forward kinematic matricies
Ts = KG3_preCompute();

% Precomputing the analytical jacobian
% [~,~,~,T_all] = KG3_FKM();
% % Computing the geometric jacobian
% J = ComputeGeometricJacobian(T_all); %%possibly being computed wrong


q0 = zeros(7,1); % initial guess
W = 0.01*eye(7);
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
configs = {};
for i=1:N
    % Extracting the desired pose at this time step
    x_des = x_ref(:,i);
    p_des = x_des(1:3);
    % phi_des = x_des(end);
    eul_des = x_des(4:end);
    
    % Solving the IKM for the desired pose
    sigma_sol = fmincon(@(sigma)obj(sigma,sigma0,W,rhoVec), sigma0, [], [], [], [], lb_alt, ub_alt,...
        @(sigma) constraints(sigma, p_des, eul_des, Ts), options);

    % Extract the actual pose after solving the inverse kinematics
    q = sigma_sol(1:7);
    % configs{i} = q;
    % [p_act, R_act, ~] = KG3_FKM_simple(q, Ts);

    transform = getTransform(gen3, q, 'end_effector_link');
    R_act = transform(1:3,1:3);
    p_act = transform(1:3,4);

    eul_act = rotm2eul(R_act);
    x_act(:,i) = [p_act; eul_act.']; 

    

    % x_act(:,i) = [p_act; eul_act(1)]; 

    % Using the previous joint configuration as the inital guess in the
    % next IKM solve
    sigma0 = [q;zeros(6,1)];
    

end


%% Plotting
figure(2)
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
plot(rad2deg(x_ref(4,:)), '--r', 'LineWidth',3)
plot(rad2deg(x_act(4,:)), 'ro', 'LineWidth', 3)
plot(rad2deg(x_ref(5,:)), '--b', 'LineWidth', 3)
plot(rad2deg(x_act(5,:)), 'bo', 'LineWidth', 3)
plot(rad2deg(x_ref(6,:)), '--g', 'LineWidth', 3)
plot(rad2deg(x_act(6,:)), 'go', 'LineWidth', 3)

ylabel('Orientation (^\circ)', 'FontSize', 18)
xlabel('Sample Point', 'FontSize', 18)
legend('\phi', '\phi *','\theta', '\theta *','\psi', '\psi *')
% legend('x*', 'x', 'y*', 'y', 'z*', 'z')

%% Animation of the configurations

% dt = ts(2) - ts(1);
% figure(5);
% for i = 1:N
% 
%     config = configs{i};
%     % Show the robot in the current configuration
%     show(gen3, config);
% 
%     % Pause to create an animation effect
%     pause(dt); % Adjust the pause duration as needed
%  end


function cost = obj(sigma, sigma0, W, rhoVec)
    
    q = sigma(1:7);
    s = sigma(8:end);
    q0 = sigma0(1:7);

    cost = (q-q0).'*W*(q-q0) + rhoVec.'*s;
end

function [cineq, ceq, gcineq] = constraints(sigma, p_des, eul_des, Ts)
    q = sigma(1:7);
    s = sigma(8:end);
    [p, R, T_all] = KG3_FKM_simple(q, Ts);
    eul = rotm2eul(R);
    
    pose = [p;eul.'];
    pose_des = [p_des;eul_des];

    % Equality constraints
    ceq = [];

    % Pose inequality constraints
    c_upper = pose - s - pose_des;
    c_lower = -pose -s + pose_des;
    cineq = [c_upper;c_lower];

    % Inequality constraint Jacobian
    phi = eul(1);
    theta = eul(2);
    % J = ComputeGeometricJacobian(T_all);
    % JA = KG3_analyticalJacobian(phi,theta, J);
    J = KG3_JGEO(T_all);
    JA = KG3_JA(phi,theta, J);

    gcineq = zeros(12, 13);
    gcineq(1:6, 1:7) = JA;         % ∂cineq1/∂q = JA
    gcineq(1:6, 8:13) = -eye(6);    % ∂cineq1/∂s = -I
    gcineq(7:12, 1:7) = -JA;        % ∂cineq2/∂q = -JA
    gcineq(7:12, 8:13) = -eye(6);    % ∂cineq2/∂s = -I
    gcineq = gcineq';

end