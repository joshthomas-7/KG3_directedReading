clear
close
clc

%% Loading the KG3 
gen3 = loadrobot("kinovaGen3", "DataFormat", "column", "Version", 2, Gravity=[0 0 -9.81]);
% gen3 = loadrobot("kinovaGen3", "DataFormat", "column", Gravity=[0 0 -9.81]);
b = gen3.Bodies;
% show(gen3);
showdetails(gen3)

% Getting the home configuration
% q = homeConfiguration(gen3);
q = randomConfiguration(gen3);
% q = [0.3919, -1.3412, -0.9480, -1.0817, -0.5078, -0.3382, 0.0125].';

% qdot = [0.1, 0.2, -0.1, 0.05, -0.15, 0.08, 0.12].';
qdot = generateRandomQdot('slow')
% qdot = zeros(7,1);



%% Robot Dynamics Simulation Script
% clear;

% Load the previously computed trajectory data
data = load('joint_space_data.mat');
q_traj = data.q_history;      % Reference joint positions
qdot_traj = data.qdot_history; % Reference joint velocities
qddot_traj = data.qddot_history; % Reference joint accelerations
x_ee_desired = data.x_ee_desired;   % Reference pose
dx_ee_desired = data.dx_ee_desired; % Reference pose velocity
ddx_ee_desired = data.ddx_ee_desired;   % Reference pose acceleration

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
dx_actual = zeros(6, Nsim);     % Actual end-effector velocity

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
    u = myKG3.inverseDynamicsControl(QD_desired, Q_actual);
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
    
    % Compute end-effector velocity using finite differences
    % This provides the "true" end-effector velocity based on actual pose changes
    if i == 1
        % For first iteration, use forward difference
        dx_actual(:,i+1) = (x_actual(:,i+1) - x_actual(:,i)) / dt;
    else
        % For subsequent iterations, use central difference for better accuracy
        dx_actual(:,i+1) = (x_actual(:,i+1) - x_actual(:,i-1)) / (2*dt);
    end
end

M = massMatrix(gen3,q_curr);
vProd = velocityProduct(gen3,q_curr,qdot_curr);
G = gravityTorque(gen3,q_curr);
% Store final control torque
u_control(:,end) = myKG3.inverseDynamicsControl([q_traj(:,end), qdot_traj(:,end), qddot_traj(:,end)], ...
                                               [q_actual(:,end), qdot_actual(:,end), qddot_actual(:,end)]);

% Update velocity for current iteration (needed for control loop)
if i > 1
    dx_curr = (x_actual(:,i) - x_actual(:,i-1)) / dt;
    dx_actual(:,i) = dx_curr;
end

fprintf('Simulation completed\n');

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
title('End-Effector Position Tracking (Joint Space Control)');
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
title('End-Effector Orientation Tracking (Joint Space Control)');
legend('\phi_{des}', '\phi_{act}', '\theta_{des}', '\theta_{act}', '\psi_{des}', '\psi_{act}', 'Location', 'best');
grid on;

% End-effector velocity tracking
figure(2);
subplot(2,1,1);
plot(time_vec, dx_ee_desired(1,:), 'b--', 'LineWidth', 2); hold on;
plot(time_vec, dx_actual(1,:), 'b-', 'LineWidth', 2);
plot(time_vec, dx_ee_desired(2,:), 'r--', 'LineWidth', 2);
plot(time_vec, dx_actual(2,:), 'r-', 'LineWidth', 2);
plot(time_vec, dx_ee_desired(3,:), 'g--', 'LineWidth', 2);
plot(time_vec, dx_actual(3,:), 'g-', 'LineWidth', 2);
ylabel('Linear Velocity (m/s)');
title('End-Effector Linear Velocity Tracking (Joint Space Control)');
legend('V_x_{des}', 'V_x_{act}', 'V_y_{des}', 'V_y_{act}', 'V_z_{des}', 'V_z_{act}', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(time_vec, rad2deg(dx_ee_desired(4,:)), 'b--', 'LineWidth', 2); hold on;
plot(time_vec, rad2deg(dx_actual(4,:)), 'b-', 'LineWidth', 2);
plot(time_vec, rad2deg(dx_ee_desired(5,:)), 'r--', 'LineWidth', 2);
plot(time_vec, rad2deg(dx_actual(5,:)), 'r-', 'LineWidth', 2);
plot(time_vec, rad2deg(dx_ee_desired(6,:)), 'g--', 'LineWidth', 2);
plot(time_vec, rad2deg(dx_actual(6,:)), 'g-', 'LineWidth', 2);
ylabel('Angular Velocity (deg/s)');
xlabel('Time (s)');
title('End-Effector Orientation Velocity Tracking (Joint Space Control)');
legend('d\phi_{des}', 'd\phi_{act}', 'd\theta_{des}', 'd\theta_{act}', 'd\psi_{des}', 'd\psi_{act}', 'Location', 'best');
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

% End-effector tracking errors
figure(7);
pos_error = x_ee_desired(1:3,:) - x_actual(1:3,:);
orient_error = x_ee_desired(4:6,:) - x_actual(4:6,:);

subplot(2,1,1);
plot(time_vec, 1000*pos_error(1,:), 'b-', 'LineWidth', 1.5); hold on;
plot(time_vec, 1000*pos_error(2,:), 'r-', 'LineWidth', 1.5);
plot(time_vec, 1000*pos_error(3,:), 'g-', 'LineWidth', 1.5);
ylabel('Position Error (mm)');
title('End-Effector Position Tracking Errors (Joint Space Control)');
legend('X Error', 'Y Error', 'Z Error', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(time_vec, rad2deg(orient_error(1,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(time_vec, rad2deg(orient_error(2,:)), 'r-', 'LineWidth', 1.5);
plot(time_vec, rad2deg(orient_error(3,:)), 'g-', 'LineWidth', 1.5);
ylabel('Orientation Error (deg)');
xlabel('Time (s)');
title('End-Effector Orientation Tracking Errors (Joint Space Control)');
legend('\phi Error', '\theta Error', '\psi Error', 'Location', 'best');
grid on;

% End-effector velocity tracking errors
figure(8);
vel_linear_error = dx_ee_desired(1:3,:) - dx_actual(1:3,:);
vel_angular_error = dx_ee_desired(4:6,:) - dx_actual(4:6,:);

subplot(2,1,1);
plot(time_vec, 1000*vel_linear_error(1,:), 'b-', 'LineWidth', 1.5); hold on;
plot(time_vec, 1000*vel_linear_error(2,:), 'r-', 'LineWidth', 1.5);
plot(time_vec, 1000*vel_linear_error(3,:), 'g-', 'LineWidth', 1.5);
ylabel('Linear Velocity Error (mm/s)');
title('End-Effector Linear Velocity Tracking Errors (Joint Space Control)');
legend('V_x Error', 'V_y Error', 'V_z Error', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(time_vec, rad2deg(vel_angular_error(1,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(time_vec, rad2deg(vel_angular_error(2,:)), 'r-', 'LineWidth', 1.5);
plot(time_vec, rad2deg(vel_angular_error(3,:)), 'g-', 'LineWidth', 1.5);
ylabel('Angular Velocity Error (deg/s)');
xlabel('Time (s)');
title('End-Effector Angular Velocity Tracking Errors (Joint Space Control)');
legend('\omega_\phi Error', '\omega_\theta Error', '\omega_\psi Error', 'Location', 'best');
grid on;

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