clear
close
clc

%% Loading the KG3 
gen3 = loadrobot("kinovaGen3", "DataFormat", "column", "Version", 1, Gravity=[0 0 -9.81]);
% gen3 = loadrobot("kinovaGen3", "DataFormat", "column", Gravity=[0 0 -9.81]);
b = gen3.Bodies;
% show(gen3);
showdetails(gen3)

% Getting the home configuration
% q = homeConfiguration(gen3);
q = randomConfiguration(gen3);
% q = [0.3919, -1.3412, -0.9480, -1.0817, -0.5078, -0.3382, 0.0125].';

% qdot = [0.1, 0.2, -0.1, 0.05, -0.15, 0.08, 0.12].';
qdot = generateRandomQdot('realistic')
% qdot = zeros(7,1);

myKG3 = KG3();
myKG3 = myKG3.FKM(q);

%% Verifying the FKM
% T = getTransform(gen3, q, 'end_effector_link')
myKG3.T_all{end}

%% Testing Jacobian computation
myKG3 = myKG3.computeJacobians();
disp('My J')
myKG3.J

% J = geometricJacobian(gen3, q, 'end_effector_link')

%% Testing the Mass Matrix

M = massMatrix(gen3, q)

myKG3 = myKG3.computeMassMatrix();
myKG3.M

%% Testing the coriolis matrix

myKG3 = myKG3.computeCoriolisMatrix(qdot);

C = myKG3.C;
estTorque = C*qdot

trueTorque = velocityProduct(gen3, q, qdot)

%% Testing Gravity Torque
myKG3 = myKG3.computeGravityTorque();
% myKG3 = myKG3.computeGravityTorqueAnalyticalSimplified();
disp('My G')
myKG3.G

G = gravityTorque(gen3, q)

%% Testing Acceleration

qddot_matlab = forwardDynamics(gen3, q, qdot)
qddot = myKG3.M\(-myKG3.C*qdot - myKG3.G)
%% Dynamics Simulation
tspan = [0, 1];                                         % 5 second simulation

Q = [q, qdot, qddot];
QD = Q;

% u = myKG3.inverseDynamicsControl(QD,Q);

%% Robot Dynamics Simulation Script
clear; clc;

% Load the previously computed trajectory data
data = load('joint_space_data.mat');
q_traj = data.q_history;      % Reference joint positions
qdot_traj = data.qdot_history; % Reference joint velocities
qddot_traj = data.qddot_history; % Reference joint accelerations

% Check if end-effector trajectory data exists
if isfield(data, 'x_ee_history')
    x_ee_desired = data.x_ee_history;    % Desired end-effector trajectory
    dx_ee_desired = data.dx_ee_history;  % Desired end-effector velocities
    fprintf('Loaded desired end-effector trajectory from saved data.\n');
else
    % If not available, compute desired end-effector trajectory from joint trajectory
    fprintf('Computing desired end-effector trajectory from joint data...\n');
    N = size(q_traj, 2);
    x_ee_desired = zeros(6, N);
    dx_ee_desired = zeros(6, N);
    
    for i = 1:N
        % Compute desired end-effector pose
        T_des = getTransform(gen3, q_traj(:,i), 'end_effector_link');
        R_des = T_des(1:3, 1:3);
        p_des = T_des(1:3, 4);
        eul_des = rotm2eul(R_des);
        x_ee_desired(:,i) = [p_des; eul_des'];
        
        % Compute desired end-effector velocity using Jacobian
        if i > 1
            J_des = geometricJacobian(gen3, q_traj(:,i), 'end_effector_link');
            dx_ee_desired(:,i) = J_des * qdot_traj(:,i);
        end
    end
    fprintf('Desired end-effector trajectory computed.\n');
end

time_vec = data.ts;

% Initialize robot and controller
myKG3 = KG3();
gen3 = loadrobot("kinovaGen3", "DataFormat", "column", "Version", 2, Gravity=[0 0 -9.81]);

%% Simulation Parameters
Nsim = length(time_vec);
dt = time_vec(2) - time_vec(1);

% Initialize actual trajectory arrays
q_actual = zeros(7, Nsim);      % Actual joint positions
qdot_actual = zeros(7, Nsim);   % Actual joint velocities
qddot_actual = zeros(7, Nsim);  % Actual joint accelerations
u_control = zeros(7, Nsim);     % Control torques

% Initialize end-effector pose arrays
x_actual = zeros(6, Nsim);      % Actual end-effector pose [x,y,z,phi,theta,psi]

% Initial conditions (start from reference trajectory)
q_actual(:,1) = q_traj(:,1);
qdot_actual(:,1) = qdot_traj(:,1);
qddot_actual(:,1) = qddot_traj(:,1);

% Compute initial end-effector pose
T_init = getTransform(gen3, q_actual(:,1), 'end_effector_link');
R_init = T_init(1:3, 1:3);
p_init = T_init(1:3, 4);
eul_init = rotm2eul(R_init);
x_actual(:,1) = [p_init; eul_init'];

% You can add some initial disturbance if desired
% q_actual(:,1) = q_traj(:,1) + 0.05*randn(7,1);  % Small random disturbance
% qdot_actual(:,1) = qdot_traj(:,1) + 0.1*randn(7,1);

%% Simulation Loop
fprintf('Running dynamics simulation...\n');

for i = 1:Nsim-1
    % Current actual state
    q_curr = q_actual(:,i);
    qdot_curr = qdot_actual(:,i);
    
    % Current actual state vector for controller
    Q_actual = [q_curr, qdot_curr, qddot_actual(:,i)];
    
    % Desired state at current time step
    QD_desired = [q_traj(:,i), qdot_traj(:,i), qddot_traj(:,i)];
    
    % Compute control torques using inverse dynamics controller
    M = massMatrix(gen3,q_curr);
    vProd = velocityProduct(gen3,q_curr,qdot_curr);
    G = gravityTorque(gen3,q_curr);
    u = myKG3.inverseDynamicsControl(QD_desired, Q_actual, M, vProd, G);
    u_control(:,i) = u;
    
    % Simulate forward dynamics to get actual acceleration
    qddot_new = forwardDynamics(gen3, q_curr, qdot_curr, u);
    qddot_actual(:,i+1) = qddot_new;
    
    % Integrate to get next velocity and position using Euler integration
    qdot_new = qdot_curr + qddot_new * dt;
    q_new = q_curr + qdot_curr * dt + 0.5 * qddot_new * dt^2;
    
    % Store results
    q_actual(:,i+1) = q_new;
    qdot_actual(:,i+1) = qdot_new;
    
    % Compute end-effector pose at new configuration
    T_new = getTransform(gen3, q_new, 'end_effector_link');
    R_new = T_new(1:3, 1:3);
    p_new = T_new(1:3, 4);
    eul_new = rotm2eul(R_new);
    x_actual(:,i+1) = [p_new; eul_new'];
    
    % Progress indicator
    if mod(i, 10) == 0
        fprintf('Progress: %d/%d (%.1f%%)\n', i, Nsim-1, 100*i/(Nsim-1));
    end
end

M = massMatrix(gen3,q_curr);
vProd = velocityProduct(gen3,q_curr,qdot_curr);
G = gravityTorque(gen3,q_curr);
% Store final control torque
u_control(:,end) = myKG3.inverseDynamicsControl([q_traj(:,end), qdot_traj(:,end), qddot_traj(:,end)], ...
                                               [q_actual(:,end), qdot_actual(:,end), qddot_actual(:,end)],...
                                               M, vProd, G);

fprintf('Simulation completed!\n');

%% Plotting Results

% End-effector pose tracking comparison
figure(1);
subplot(2,1,1);
plot(time_vec, x_ee_desired(1,:), 'b--', 'LineWidth', 2); hold on;
plot(time_vec, x_actual(1,:), 'b-', 'LineWidth', 2);
plot(time_vec, x_ee_desired(2,:), 'r--', 'LineWidth', 2);
plot(time_vec, x_actual(2,:), 'r-', 'LineWidth', 2);
plot(time_vec, x_ee_desired(3,:), 'g--', 'LineWidth', 2);
plot(time_vec, x_actual(3,:), 'g-', 'LineWidth', 2);
ylabel('Position (m)');
title('End-Effector Position Tracking');
legend('X_{des}', 'X_{act}', 'Y_{des}', 'Y_{act}', 'Z_{des}', 'Z_{act}', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(time_vec, rad2deg(x_ee_desired(4,:)), 'b--', 'LineWidth', 2); hold on;
plot(time_vec, rad2deg(x_actual(4,:)), 'b-', 'LineWidth', 2);
plot(time_vec, rad2deg(x_ee_desired(5,:)), 'r--', 'LineWidth', 2);
plot(time_vec, rad2deg(x_actual(5,:)), 'r-', 'LineWidth', 2);
plot(time_vec, rad2deg(x_ee_desired(6,:)), 'g--', 'LineWidth', 2);
plot(time_vec, rad2deg(x_actual(6,:)), 'g-', 'LineWidth', 2);
ylabel('Orientation (deg)');
xlabel('Time (s)');
title('End-Effector Orientation Tracking');
legend('\phi_{des}', '\phi_{act}', '\theta_{des}', '\theta_{act}', '\psi_{des}', '\psi_{act}', 'Location', 'best');
grid on;

% Joint position tracking
figure(4);
for j = 1:7
    subplot(4,2,j);
    plot(time_vec, rad2deg(q_traj(j,:)), 'b--', 'LineWidth', 2); hold on;
    plot(time_vec, rad2deg(q_actual(j,:)), 'r-', 'LineWidth', 1.5);
    ylabel(sprintf('Joint %d (deg)', j));
    if j == 7
        xlabel('Time (s)');
    end
    legend('Reference', 'Actual', 'Location', 'best');
    grid on;
    title(sprintf('Joint %d Position Tracking', j));
end
sgtitle('Joint Position Tracking Performance');

% Joint velocity tracking
figure(5);
for j = 1:7
    subplot(4,2,j);
    plot(time_vec, rad2deg(qdot_traj(j,:)), 'b--', 'LineWidth', 2); hold on;
    plot(time_vec, rad2deg(qdot_actual(j,:)), 'r-', 'LineWidth', 1.5);
    ylabel(sprintf('Joint %d (deg/s)', j));
    if j == 7
        xlabel('Time (s)');
    end
    legend('Reference', 'Actual', 'Location', 'best');
    grid on;
    title(sprintf('Joint %d Velocity Tracking', j));
end
sgtitle('Joint Velocity Tracking Performance');

% Control torques
figure(6);
for j = 1:7
    subplot(4,2,j);
    plot(time_vec, u_control(j,:), 'g-', 'LineWidth', 1.5);
    ylabel(sprintf('Torque %d (Nm)', j));
    if j == 7
        xlabel('Time (s)');
    end
    grid on;
    title(sprintf('Joint %d Control Torque', j));
end
sgtitle('Control Torques');

%% Save simulation results
simulation_results = struct();
simulation_results.time_vec = time_vec;
simulation_results.q_reference = q_traj;
simulation_results.qdot_reference = qdot_traj;
simulation_results.qddot_reference = qddot_traj;
simulation_results.q_actual = q_actual;
simulation_results.qdot_actual = qdot_actual;
simulation_results.qddot_actual = qddot_actual;
simulation_results.x_ee_desired = x_ee_desired;     % Desired end-effector trajectory
simulation_results.x_actual = x_actual;            % Actual end-effector poses
simulation_results.u_control = u_control;

save('dynamics_simulation_results.mat', '-struct', 'simulation_results');
fprintf('\nSimulation results saved to dynamics_simulation_results.mat\n');

%% Animation
% 
% fprintf('\nStarting animation...\n');
% figure(7);
% for i = 1:5:Nsim  % Animate every 5th frame for speed
%     clf;
%     show(gen3, q_actual(:,i));
%     title(sprintf('Robot Motion Animation - Time: %.2f s', time_vec(i)));
%     drawnow;
%     pause(0.1);
% end

%% Functions
function qdot = generateRandomQdot(method, varargin)

    % Generates random joint velocities for the 7-DOF KG3 robot
    % 
    % Usage:
    %   qdot = generateRandomQdot('uniform')           % Uniform [-1, 1] rad/s
    %   qdot = generateRandomQdot('uniform', max_vel)  % Uniform [-max_vel, max_vel]
    %   qdot = generateRandomQdot('gaussian')          % Gaussian with std=0.5
    %   qdot = generateRandomQdot('gaussian', sigma)   % Gaussian with std=sigma
    %   qdot = generateRandomQdot('realistic')         % Realistic joint velocities
    %   qdot = generateRandomQdot('fast')              % Fast motion velocities
    %   qdot = generateRandomQdot('slow')              % Slow motion velocities
    
    if nargin < 1
        method = 'uniform';
    end
    
    N = 7;  % Number of joints for KG3
    
    switch lower(method)
        case 'uniform'
            % Uniform distribution
            if nargin >= 2
                max_vel = varargin{1};
            else
                max_vel = 1.0;  % Default max velocity in rad/s
            end
            qdot = (2 * rand(N, 1) - 1) * max_vel;
            
        case 'gaussian'
            % Gaussian (normal) distribution
            if nargin >= 2
                sigma = varargin{1};
            else
                sigma = 0.5;  % Default standard deviation in rad/s
            end
            qdot = sigma * randn(N, 1);
            
        case 'realistic'
            % Realistic joint velocities based on typical robot operation
            % Different joints have different typical velocity ranges
            max_velocities = [2.0, 2.0, 2.5, 2.5, 3.0, 3.0, 3.0];  % rad/s
            qdot = zeros(N, 1);
            for i = 1:N
                qdot(i) = (2 * rand() - 1) * max_velocities(i);
            end
            
        case 'fast'
            % Fast motion - higher velocities
            max_vel = 3.0;  % rad/s
            qdot = (2 * rand(N, 1) - 1) * max_vel;
            
        case 'slow'
            % Slow motion - lower velocities
            max_vel = 0.2;  % rad/s
            qdot = (2 * rand(N, 1) - 1) * max_vel;
            
        case 'mixed'
            % Mixed velocities - some joints fast, some slow
            fast_joints = randperm(N, 3);  % Randomly select 3 joints to be fast
            qdot = 0.2 * (2 * rand(N, 1) - 1);  % Start with slow velocities
            qdot(fast_joints) = 2.0 * (2 * rand(3, 1) - 1);  % Make selected joints fast
            
        otherwise
            error('Unknown method. Use: uniform, gaussian, realistic, fast, slow, mixed');
    end
    
    % Display the generated velocities
    fprintf('Generated joint velocities (rad/s):\n');
    fprintf('qdot = [');
    for i = 1:N
        if i < N
            fprintf('%.4f, ', qdot(i));
        else
            fprintf('%.4f', qdot(i));
        end
    end
    fprintf('];\n');
    
    % Convert to degrees for reference
    qdot_deg = qdot * 180/pi;
    fprintf('In degrees/s: [');
    for i = 1:N
        if i < N
            fprintf('%.1f, ', qdot_deg(i));
        else
            fprintf('%.1f', qdot_deg(i));
        end
    end
    fprintf(']\n\n');
end

% function simulationResults = simulateKG3FreeMotion(robot, q0, qdot0, tspan)
%     % Simulates free motion dynamics of the KG3 robot (no control torques)
%     % 
%     % Inputs:
%     %   robot - KG3 robot object
%     %   q0 - Initial joint positions (7x1) [rad]
%     %   qdot0 - Initial joint velocities (7x1) [rad/s]
%     %   tspan - Time span [t_start, t_end] or time vector [s]
%     %
%     % Outputs:
%     %   simulationResults - Structure containing time, positions, velocities
% 
% 
%     % Set up time vector
%     if length(tspan) == 2
%         t_sim = linspace(tspan(1), tspan(2), 1000);
%     else
%         t_sim = tspan;
%     end
% 
%     % Initial state vector [q; qdot]
%     x0 = [q0(:); qdot0(:)];
% 
%     % Define dynamics function
%     dynamics_func = @(t, x) robotFreeMotionDynamics(t, x, robot);
% 
%     % Solve ODE
%     fprintf('Starting free motion dynamics simulation...\n');
%     fprintf('Time span: %.2f to %.2f seconds\n', t_sim(1), t_sim(end));
% 
%     options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 0.01);
%     [t_out, x_out] = ode45(dynamics_func, t_sim, x0, options);
% 
%     % Extract results
%     q_out = x_out(:, 1:7);
%     qdot_out = x_out(:, 8:14);
% 
%     % Package results
%     simulationResults.time = t_out;
%     simulationResults.positions = q_out;
%     simulationResults.velocities = qdot_out;
%     simulationResults.robot = robot;
% 
%     fprintf('Simulation complete!\n');
% 
%     % Plot results
%     plotSimulationResults(simulationResults);
% end
% 
% function xdot = robotFreeMotionDynamics(t, x, robot)
%     % Robot free motion dynamics function for ODE solver
%     % Implements: M(q)*qddot + C(q,qdot)*qdot + G(q) = 0 (no applied torques)
% 
%     % Extract state
%     q = x(1:7);
%     qdot = x(8:14);
% 
%     % Update robot state and compute dynamics
%     robot.q = q;
%     robot.qdot = qdot;
%     robot = robot.FKM(q);
%     robot = robot.computeJacobians();
%     robot = robot.computeMassMatrix();
%     robot = robot.computeCoriolisMatrix(qdot);
%     robot = robot.computeGravityTorque();
% 
%     % Forward dynamics with zero applied torques: qddot = M^(-1) * (-C*qdot - G)
%     qddot = robot.M \ (-robot.C * qdot - robot.G);
% 
%     % State derivative
%     xdot = [qdot; qddot];
% end
% 
% function plotSimulationResults(results)
%     % Plot free motion simulation results
% 
%     figure('Position', [100, 100, 1200, 600]);
% 
%     % Plot joint positions
%     subplot(2, 1, 1);
%     plot(results.time, results.positions * 180/pi);
%     title('Joint Positions - Free Motion');
%     xlabel('Time (s)');
%     ylabel('Angle (degrees)');
%     legend({'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7'}, ...
%            'Location', 'eastoutside');
%     grid on;
% 
%     % Plot joint velocities  
%     subplot(2, 1, 2);
%     plot(results.time, results.velocities * 180/pi);
%     title('Joint Velocities - Free Motion');
%     xlabel('Time (s)');
%     ylabel('Velocity (deg/s)');
%     legend({'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7'}, ...
%            'Location', 'eastoutside');
%     grid on;
% 
%     sgtitle('KG3 Robot Free Motion Dynamics Simulation');
% end