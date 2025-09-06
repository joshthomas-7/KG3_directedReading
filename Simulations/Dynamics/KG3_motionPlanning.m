%%% Joshua Thomas
%%% C3376353
clear
close all
clc

%%% This script solves the IKM problem for a custom trajectory. It aims to
%%% achieve pose, velocity, and acceleration tracking using the derived
%%% kinematic model for the KG3.

%% Initialising the KG3 class
myKG3 = KG3();

%% Loading the KG3 
gen3 = loadrobot("kinovaGen3", "DataFormat", "column", "Version", 2, Gravity=[0 0 -9.81]);
b = gen3.Bodies;
% show(gen3);
showdetails(gen3)

%% Defining a trajectory with quintic polynomials
timePoints = [0 3 6];

% Position waypoints
% waypoints = [0.579  -0.004 0.4338 pi/2 0 pi/2; 
%              0.579  -0.20 0.65 pi/2 deg2rad(45) pi/2;
%              0.75  -0.004 0.4338 pi/2 0 pi/2]';
waypoints = [0.579  -0.004 0.4338 pi/2 0 pi/2; 
             0.200  -0.20 0.65 pi/2 pi/4 pi/4;
             0.300  -0.004 0.4338 pi/2 pi/3 0]';

% Desired velocities at waypoints
velocities = [0.00 0.00 0.00 0.00 0 0; 
              0.00  0.00 0.00 0.00 0 0;
              0.00  0.00 0.00 0.00 0 0.00]';

% Desired accelerations at waypoints
% [ax ay az a_phi a_theta a_psi] for each waypoint
accelerations = [0.00  0.00  0.00  0.00 0.00 0.00;  % Start: zero acceleration
                 0.00  0.00  0.00  0.00 0.00 0.00;  % Waypoint 4
                 0.00  0.00  0.00  0.00 0.00 0.00]'; % End: zero acceleration

ts = linspace(0,6,75);

% Compute quintic polynomial trajectory
[x_ref, dx_ref, ddx_ref] = quinticpolytraj(waypoints, timePoints, ts, ...
    'VelocityBoundaryCondition', velocities, ...
    'AccelerationBoundaryCondition', accelerations);

% Plot trajectory including accelerations
figure(1);
subplot(2,1,1)
plot(ts, x_ref(1,:), 'r', ts, x_ref(2,:), 'g', ts, x_ref(3,:), 'b', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Position (m)'); title('Cartesian Position Trajectory');
legend('x','y','z'); grid on;

subplot(2,1,2)
plot(ts, rad2deg(x_ref(4,:)), 'r', ts,rad2deg(x_ref(5,:)), 'g', ts, rad2deg(x_ref(6,:)), 'b', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Orientation (deg)'); title('Orientation Trajectory');
legend('\phi','\theta','\psi'); grid on;

figure(2);
% subplot(3,1,3)
plot(ts, dx_ref(1,:), 'r', ts, dx_ref(2,:), 'g', ts, dx_ref(3,:), 'b', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Velocity (m/s)'); title('Linear Velocity Trajectory');
legend('dx','dy','dz'); grid on;

% Plot accelerations
figure(3);
subplot(2,1,1)
plot(ts, ddx_ref(1,:), 'r', ts, ddx_ref(2,:), 'g', ts, ddx_ref(3,:), 'b', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Acceleration (m/s²)'); title('Linear Acceleration Trajectory');
legend('ddx','ddy','ddz'); grid on;

subplot(2,1,2)
plot(ts, rad2deg(ddx_ref(4,:)), 'r', ts, rad2deg(ddx_ref(5,:)), 'g', ts, rad2deg(ddx_ref(6,:)), 'b', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Angular Acceleration (deg/s²)'); title('Angular Acceleration Trajectory');
legend('dd\phi','dd\theta','dd\psi'); grid on;

%% Setting up the IKM optimisation problem
q0 = zeros(7,1); % initial guess
myKG3.W = 1e-2*eye(7);
myKG3.V = 1e-1*eye(7);

% myKG3.rhoVec = [1.5e2*ones(6,1);2.5e1*ones(6,1)];
myKG3.rhoVec = [1.5e3*ones(6,1);5e1*ones(6,1)];
options = optimoptions('fmincon', 'Display', 'none');
lb = -pi*ones(7,1);
ub = pi*ones(7,1);

sigma0 = zeros(7+6+6,1);
lb_alt = [lb;zeros(6+6,1)];
ub_alt = [ub;1e3*ones(6+6,1)];

myKG3.dt = ts(2) - ts(1);

%% Simulating the trajectory Tracking with velocity validation

N = width(x_ref);
x_act = zeros(size(x_ref));
dx_act = zeros(size(x_ref));
ddx_act = zeros(size(x_ref)); % Store computed accelerations
dx_finite_diff = zeros(size(x_ref)); % Finite difference validation
ddx_finite_diff = zeros(size(x_ref)); % Finite difference acceleration validation

% Initialize storage for joint space variables
q_history = zeros(7, N);       % Joint positions
qdot_history = zeros(7, N);    % Joint velocities  
qddot_history = zeros(7, N);   % Joint accelerations

% Initialize storage for end-effector pose variables
x_ee_history = zeros(6, N);    % End-effector positions and orientations [x,y,z,phi,theta,psi]
dx_ee_history = zeros(6, N);   % End-effector velocities
ddx_ee_history = zeros(6, N);  % End-effector accelerations

configs = {};

firstTime = true;
qdot_prev = zeros(7,1);  % Store previous joint velocity

for i=1:N
    % Extracting the desired pose at this time step
    x_des = x_ref(:,i);
    dx_des = dx_ref(:,i);
    ddx_des = ddx_ref(:,i); % Desired acceleration
    
    % Solving the IKM for the desired pose
    sigma_sol = fmincon(@(sigma)myKG3.IKMCost(sigma, sigma0, qdot_prev), sigma0, [], [], [], [], lb_alt, ub_alt,...
        @(sigma) myKG3.IKMConstraints(sigma, sigma0, x_des, dx_des, firstTime), options);

    firstTime = false;

    % Extract the actual pose after solving the inverse kinematics
    q = sigma_sol(1:7);
    configs{i} = q;
    
    % Store joint position
    q_history(:,i) = q;

    % Compute joint velocity
    if i == 1
        qdot = zeros(7,1);  % Initial velocity is zero
    else
        qdot = (q - q_history(:,i-1))/myKG3.dt;
    end
    qdot_history(:,i) = qdot;
    
    % Compute joint acceleration
    if i == 1
        qddot = zeros(7,1);  % Initial acceleration is zero
    else
        qddot = (qdot - qdot_prev)/myKG3.dt;
    end
    qddot_history(:,i) = qddot;

    % Compute end-effector pose using forward kinematics
    transform = getTransform(gen3, q, 'end_effector_link');
    R_act = transform(1:3,1:3);
    p_act = transform(1:3,4);
    eul_act = rotm2eul(R_act);
    
    % Store actual end-effector pose
    x_act(:,i) = [p_act; eul_act.'];
    x_ee_history(:,i) = [p_act; eul_act.'];

    % Compute velocities using Jacobian method
    myKG3 = myKG3.FKM(q);
    myKG3 = myKG3.computeJacobians();
    myKG3 = myKG3.computeAnalyticalJacobian();
    myKG3 = myKG3.computeAnalyticalHessian();
    
    % Compute end-effector velocity using Jacobian
    dx_ee = myKG3.JA*qdot;
    dx_act(:,i) = dx_ee;
    dx_ee_history(:,i) = dx_ee;
    
    % Compute acceleration using analytical Hessian
    if i > 1
        ddx_ee = myKG3.computePoseAcceleration(qdot, qddot);
        ddx_act(:,i) = ddx_ee;
        ddx_ee_history(:,i) = ddx_ee;
    end

    % Compute finite difference velocities and accelerations for validation
    if i > 1
        dx_finite_diff(:,i) = (x_act(:,i) - x_act(:,i-1)) / myKG3.dt;
        
        if i > 2
            ddx_finite_diff(:,i) = (dx_finite_diff(:,i) - dx_finite_diff(:,i-1)) / myKG3.dt;
        end
    end

    % Update previous joint velocity for next iteration
    qdot_prev = qdot;
    
    % Using the previous joint configuration as the initial guess in the
    % next IKM solve
    sigma0 = [q;zeros(6+6,1)];
end

%% Plotting Results
figure(4)
subplot(2,1,1)
hold on
plot(x_ref(1,:), 'r', 'LineWidth',3)
plot(x_act(1,:), 'ro', 'LineWidth', 3)
plot(x_ref(2,:), 'b', 'LineWidth', 3)
plot(x_act(2,:), 'bo', 'LineWidth', 3)
plot(x_ref(3,:), 'g', 'LineWidth', 3)
plot(x_act(3,:), 'go', 'LineWidth', 3)

ylabel('Position (m)', 'FontSize', 18)
xlabel('Sample Point', 'FontSize', 18)
title('Position Tracking Performance')
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
title('Orientation Tracking Performance')
legend('\phi*', '\phi','\theta*', '\theta','\psi*', '\psi')

%% Velocity Validation Plots
figure(5)
subplot(3,2,1);
plot(ts, dx_act(1,:), 'r-', 'LineWidth', 2); hold on;
plot(ts, dx_finite_diff(1,:), 'b--', 'LineWidth', 2);
plot(ts, dx_ref(1,:), 'k:', 'LineWidth', 2);
ylabel('dx (m/s)'); title('X-velocity');
legend('Jacobian', 'Finite Diff', 'Desired'); grid on;
xlim([0.5,6])

subplot(3,2,3);
plot(ts, dx_act(2,:), 'r-', 'LineWidth', 2); hold on;
plot(ts, dx_finite_diff(2,:), 'b--', 'LineWidth', 2);
plot(ts, dx_ref(2,:), 'k:', 'LineWidth', 2);
ylabel('dy (m/s)'); title('Y-velocity');
legend('Jacobian', 'Finite Diff', 'Desired'); grid on;
xlim([0.5,6])

subplot(3,2,5);
plot(ts, dx_act(3,:), 'r-', 'LineWidth', 2); hold on;
plot(ts, dx_finite_diff(3,:), 'b--', 'LineWidth', 2);
plot(ts, dx_ref(3,:), 'k:', 'LineWidth', 2);
ylabel('dz (m/s)'); xlabel('Time (s)'); title('Z-velocity');
legend('Jacobian', 'Finite Diff', 'Desired'); grid on;
xlim([0.5,6])

subplot(3,2,2);
plot(ts, rad2deg(dx_act(4,:)), 'r-', 'LineWidth', 2); hold on;
plot(ts, rad2deg(dx_finite_diff(4,:)), 'b--', 'LineWidth', 2);
plot(ts, rad2deg(dx_ref(4,:)), 'k:', 'LineWidth', 2);
ylabel('d\phi (deg/s)'); title('\phi-velocity');
legend('Jacobian', 'Finite Diff', 'Desired'); grid on;
xlim([0.5,6])

subplot(3,2,4);
plot(ts, rad2deg(dx_act(5,:)), 'r-', 'LineWidth', 2); hold on;
plot(ts, rad2deg(dx_finite_diff(5,:)), 'b--', 'LineWidth', 2);
plot(ts, rad2deg(dx_ref(5,:)), 'k:', 'LineWidth', 2);
ylabel('d\theta (deg/s)'); title('\theta-velocity');
legend('Jacobian', 'Finite Diff', 'Desired'); grid on;
xlim([0.5,6])

subplot(3,2,6);
plot(ts, rad2deg(dx_act(6,:)), 'r-', 'LineWidth', 2); hold on;
plot(ts, rad2deg(dx_finite_diff(6,:)), 'b--', 'LineWidth', 2);
plot(ts, rad2deg(dx_ref(6,:)), 'k:', 'LineWidth', 2);
ylabel('d\psi (deg/s)'); xlabel('Time (s)'); title('\psi-velocity');
legend('Jacobian', 'Finite Diff', 'Desired'); grid on;
xlim([0.5,6])

%% Acceleration Comparison Plots
figure(6)
subplot(3,2,1);
plot(ts, ddx_act(1,:), 'r-', 'LineWidth', 2); hold on;
plot(ts, ddx_finite_diff(1,:), 'b--', 'LineWidth', 2);
plot(ts, ddx_ref(1,:), 'k:', 'LineWidth', 2);
ylabel('ddx (m/s²)'); title('X-acceleration');
legend('Jacobian', 'Finite Diff', 'Desired'); grid on;

subplot(3,2,3);
plot(ts, ddx_act(2,:), 'r-', 'LineWidth', 2); hold on;
plot(ts, ddx_finite_diff(2,:), 'b--', 'LineWidth', 2);
plot(ts, ddx_ref(2,:), 'k:', 'LineWidth', 2);
ylabel('ddy (m/s²)'); title('Y-acceleration');
legend('Jacobian', 'Finite Diff', 'Desired'); grid on;

subplot(3,2,5);
plot(ts, ddx_act(3,:), 'r-', 'LineWidth', 2); hold on;
plot(ts, ddx_finite_diff(3,:), 'b--', 'LineWidth', 2);
plot(ts, ddx_ref(3,:), 'k:', 'LineWidth', 2);
ylabel('ddz (m/s²)'); xlabel('Time (s)'); title('Z-acceleration');
legend('Jacobian', 'Finite Diff', 'Desired'); grid on;

subplot(3,2,2);
plot(ts, rad2deg(ddx_act(4,:)), 'r-', 'LineWidth', 2); hold on;
plot(ts, rad2deg(ddx_finite_diff(4,:)), 'b--', 'LineWidth', 2);
plot(ts, rad2deg(ddx_ref(4,:)), 'k:', 'LineWidth', 2);
ylabel('dd\phi (deg/s²)'); title('\phi-acceleration');
legend('Jacobian', 'Finite Diff', 'Desired'); grid on;

subplot(3,2,4);
plot(ts, rad2deg(ddx_act(5,:)), 'r-', 'LineWidth', 2); hold on;
plot(ts, rad2deg(ddx_finite_diff(5,:)), 'b--', 'LineWidth', 2);
plot(ts, rad2deg(ddx_ref(5,:)), 'k:', 'LineWidth', 2);
ylabel('dd\theta (deg/s²)'); title('\theta-acceleration');
legend('Jacobian', 'Finite Diff', 'Desired'); grid on;

subplot(3,2,6);
plot(ts, rad2deg(ddx_act(6,:)), 'r-', 'LineWidth', 2); hold on;
plot(ts, rad2deg(ddx_finite_diff(6,:)), 'b--', 'LineWidth', 2);
plot(ts, rad2deg(ddx_ref(6,:)), 'k:', 'LineWidth', 2);
ylabel('dd\psi (deg/s²)'); xlabel('Time (s)'); title('\psi-acceleration');
legend('Jacobian', 'Finite Diff', 'Desired'); grid on;


%% Save joint space and end-effector data for further analysis
save('joint_space_data.mat', 'q_history', 'qdot_history', 'qddot_history', ...
     'x_ee_history', 'dx_ee_history', 'ddx_ee_history', 'ts');

fprintf('\nJoint space and end-effector data saved to joint_space_data.mat\n');

%% Animation of the configurations
% dt = ts(2) - ts(1);
% for i = 1:N
%     config = configs{i};
%     % Show the robot in the current configuration
%     figure(9)
%     show(gen3, config);
%     % Pause to create an animation effect
%     pause(dt); % Adjust the pause duration as needed
% end