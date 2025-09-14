clear
close
clc

%% Loading the KG3 
gen3 = loadrobot("kinovaGen3", "DataFormat", "column", "Version", 2, Gravity=[0 0 -9.81]);
b = gen3.Bodies;
showdetails(gen3)

% Getting the home configuration
q = randomConfiguration(gen3);
qdot = generateRandomQdot('slow');

myKG3 = KG3();
myKG3 = myKG3.FKM(q);

%% Robot Dynamics Simulation Script with Operational Space Control

% Load the previously computed trajectory data
data = load('joint_space_data.mat');
q_traj = data.q_history;      % Reference joint positions
qdot_traj = data.qdot_history; % Reference joint velocities
qddot_traj = data.qddot_history; % Reference joint accelerations

% Compute desired end-effector trajectory from joint trajectory
fprintf('Computing desired end-effector trajectory from joint data...\n');
N = size(q_traj, 2);
x_ee_desired = zeros(6, N);
dx_ee_desired = zeros(6, N);
ddx_ee_desired = zeros(6, N);

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
        % Convert geometric to analytical Jacobian format
        phi = eul_des(1);
        theta = eul_des(2);
        T_analytical = [0 -sin(phi)  cos(phi)*cos(theta);
                       0  cos(phi)  sin(phi)*cos(theta);
                       1  0        -sin(theta)];
        TA = [eye(3) zeros(3); zeros(3) T_analytical];
        
        % Reorder geometric Jacobian to [linear; angular] format
        J_reordered = [J_des(4:6,:); J_des(1:3,:)];
        JA_des = TA \ J_reordered;
        
        dx_ee_desired(:,i) = JA_des * qdot_traj(:,i);
    end
    
    % Compute desired end-effector acceleration (simplified)
    if i > 2
        dt = data.ts(i) - data.ts(i-1);
        ddx_ee_desired(:,i) = (dx_ee_desired(:,i) - dx_ee_desired(:,i-1)) / dt;
    end
end

fprintf('Desired end-effector trajectory computed.\n');

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

% Compute initial end-effector pose and velocity
T_init = getTransform(gen3, q_actual(:,1), 'end_effector_link');
R_init = T_init(1:3, 1:3);
p_init = T_init(1:3, 4);
eul_init = rotm2eul(R_init);
x_actual(:,1) = [p_init; eul_init'];

% Compute initial end-effector velocity
J_init = geometricJacobian(gen3, q_actual(:,1), 'end_effector_link');
phi = eul_init(1);
theta = eul_init(2);
T_analytical = [0 -sin(phi)  cos(phi)*cos(theta);
               0  cos(phi)  sin(phi)*cos(theta);
               1  0        -sin(theta)];
TA = [eye(3) zeros(3); zeros(3) T_analytical];
J_reordered = [J_init(4:6,:); J_init(1:3,:)];
JA_init = TA \ J_reordered;
dx_actual(:,1) = JA_init * qdot_actual(:,1);

%% Simulation Loop
fprintf('Running operational space dynamics simulation...\n');

for i = 1:Nsim-1
    % Current actual state
    q_curr = q_actual(:,i);
    qdot_curr = qdot_actual(:,i);
    x_curr = x_actual(:,i);
    dx_curr = dx_actual(:,i);
    
    % Current actual state vectors for controller
    Q_actual = [q_curr, qdot_curr, qddot_actual(:,i)];
    X_actual = [x_curr, dx_curr, zeros(6,1)]; % acceleration will be computed
    
    % Desired end-effector state at current time step
    XD_desired = [x_ee_desired(:,i), dx_ee_desired(:,i), ddx_ee_desired(:,i)];
    
    % Compute control torques using operational space inverse dynamics controller
    u = myKG3.inverseDynamicsControl_OPSpace(XD_desired, X_actual, Q_actual);
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
    
    % Compute end-effector velocity at new configuration
    J_new = geometricJacobian(gen3, q_new, 'end_effector_link');
    phi_new = eul_new(1);
    theta_new = eul_new(2);
    T_analytical_new = [0 -sin(phi_new)  cos(phi_new)*cos(theta_new);
                       0  cos(phi_new)  sin(phi_new)*cos(theta_new);
                       1  0            -sin(theta_new)];
    TA_new = [eye(3) zeros(3); zeros(3) T_analytical_new];
    J_reordered_new = [J_new(4:6,:); J_new(1:3,:)];
    JA_new = TA_new \ J_reordered_new;
    dx_actual(:,i+1) = JA_new * qdot_new;
end

% Store final control torque
q_final = q_actual(:,end);
qdot_final = qdot_actual(:,end);
x_final = x_actual(:,end);
dx_final = dx_actual(:,end);

Q_final = [q_final, qdot_final, qddot_actual(:,end)];
X_final = [x_final, dx_final, zeros(6,1)];
XD_final = [x_ee_desired(:,end), dx_ee_desired(:,end), ddx_ee_desired(:,end)];

u_control(:,end) = myKG3.inverseDynamicsControl_OPSpace(XD_final, X_final, Q_final);

fprintf('Operational space simulation completed\n');

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
title('End-Effector Position Tracking (Operational Space Control)');
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
title('End-Effector Orientation Tracking (Operational Space Control)');
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
title('End-Effector Linear Velocity Tracking (Operational Space Control)');
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
title('End-Effector Angular Velocity Tracking (Operational Space Control)');
legend('\omega_\phi_{des}', '\omega_\phi_{act}', '\omega_\theta_{des}', '\omega_\theta_{act}', '\omega_\psi_{des}', '\omega_\psi_{act}', 'Location', 'best');
grid on;

% Control torques
figure(3);
for j = 1:7
    subplot(4,2,j);
    plot(time_vec, u_control(j,:), 'g-', 'LineWidth', 1.5);
    ylabel(sprintf('Torque %d (Nm)', j));
    if j == 7
        xlabel('Time (s)');
    end
    grid on;
    title(sprintf('Joint %d Control Torque (Op. Space)', j));
end
sgtitle('Control Torques (Operational Space Control)');

% End-effector tracking errors
figure(4);
pos_error = x_ee_desired(1:3,:) - x_actual(1:3,:);
orient_error = x_ee_desired(4:6,:) - x_actual(4:6,:);

subplot(2,1,1);
plot(time_vec, 1000*pos_error(1,:), 'b-', 'LineWidth', 1.5); hold on;
plot(time_vec, 1000*pos_error(2,:), 'r-', 'LineWidth', 1.5);
plot(time_vec, 1000*pos_error(3,:), 'g-', 'LineWidth', 1.5);
ylabel('Position Error (mm)');
title('End-Effector Position Tracking Errors');
legend('X Error', 'Y Error', 'Z Error', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(time_vec, rad2deg(orient_error(1,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(time_vec, rad2deg(orient_error(2,:)), 'r-', 'LineWidth', 1.5);
plot(time_vec, rad2deg(orient_error(3,:)), 'g-', 'LineWidth', 1.5);
ylabel('Orientation Error (deg)');
xlabel('Time (s)');
title('End-Effector Orientation Tracking Errors');
legend('\phi Error', '\theta Error', '\psi Error', 'Location', 'best');
grid on;

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
simulation_results.dx_ee_desired = dx_ee_desired;   % Desired end-effector velocities
simulation_results.ddx_ee_desired = ddx_ee_desired; % Desired end-effector accelerations
simulation_results.x_actual = x_actual;            % Actual end-effector poses
simulation_results.dx_actual = dx_actual;          % Actual end-effector velocities
simulation_results.u_control = u_control;
simulation_results.control_type = 'operational_space';

save('operational_space_simulation_results.mat', '-struct', 'simulation_results');
fprintf('\nOperational space simulation results saved to operational_space_simulation_results.mat\n');

%% Functions
function qdot = generateRandomQdot(method, varargin)
    % Generates random joint velocities for the 7-DOF KG3 robot
    if nargin < 1
        method = 'uniform';
    end
    
    N = 7;  % Number of joints for KG3
    
    switch lower(method)
        case 'uniform'
            if nargin >= 2
                max_vel = varargin{1};
            else
                max_vel = 1.0;
            end
            qdot = (2 * rand(N, 1) - 1) * max_vel;
            
        case 'gaussian'
            if nargin >= 2
                sigma = varargin{1};
            else
                sigma = 0.5;
            end
            qdot = sigma * randn(N, 1);
            
        case 'realistic'
            max_velocities = [2.0, 2.0, 2.5, 2.5, 3.0, 3.0, 3.0];
            qdot = zeros(N, 1);
            for i = 1:N
                qdot(i) = (2 * rand() - 1) * max_velocities(i);
            end
            
        case 'fast'
            max_vel = 3.0;
            qdot = (2 * rand(N, 1) - 1) * max_vel;
            
        case 'slow'
            max_vel = 0.2;
            qdot = (2 * rand(N, 1) - 1) * max_vel;
            
        case 'mixed'
            fast_joints = randperm(N, 3);
            qdot = 0.2 * (2 * rand(N, 1) - 1);
            qdot(fast_joints) = 2.0 * (2 * rand(3, 1) - 1);
            
        otherwise
            error('Unknown method. Use: uniform, gaussian, realistic, fast, slow, mixed');
    end
    
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