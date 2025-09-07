clear
close
clc

%% Configuration
numConfigurations = 10;  % Number of different robot configurations to test

% Initialize storage arrays
frobenius_errors = zeros(numConfigurations, 1);
all_configs = zeros(numConfigurations, 7);
all_qdots = zeros(numConfigurations, 7);
all_qddots_matlab = zeros(numConfigurations, 7);
all_qddots_mine = zeros(numConfigurations, 7);
all_coriolis_est = zeros(numConfigurations, 7);
all_coriolis_true = zeros(numConfigurations, 7);
all_gravity_est = zeros(numConfigurations, 7);
all_gravity_true = zeros(numConfigurations, 7);

%% Loading the KG3
gen3 = loadrobot("kinovaGen3", "DataFormat", "column", "Version", 2, Gravity=[0 0 -9.81]);
myKG3 = KG3();

fprintf('Testing %d different robot configurations...\n', numConfigurations);

%% Loop through different configurations
for config_idx = 1:numConfigurations
    fprintf('Configuration %d/%d\n', config_idx, numConfigurations);
    
    % Generate random configuration and velocity
    q = randomConfiguration(gen3);
    qdot = generateRandomQdot('mixed'); 
    
    % Store configuration and velocity
    all_configs(config_idx, :) = q';
    all_qdots(config_idx, :) = qdot';
    
    % Update robot models
    myKG3 = myKG3.FKM(q);
    
    %% Mass Matrix Comparison
    M_matlab = massMatrix(gen3, q);
    myKG3 = myKG3.computeMassMatrix();
    M_mine = myKG3.M;
    
    % Calculate Frobenius norm of difference
    frobenius_errors(config_idx) = norm(M_matlab - M_mine, 'fro');
    
    %% Coriolis/Centrifugal Forces
    myKG3 = myKG3.computeCoriolisMatrix(qdot);
    coriolis_est = myKG3.C * qdot;
    coriolis_true = velocityProduct(gen3, q, qdot);
    
    all_coriolis_est(config_idx, :) = coriolis_est';
    all_coriolis_true(config_idx, :) = coriolis_true';
    
    %% Gravity Torques
    myKG3 = myKG3.computeGravityTorque();
    gravity_est = myKG3.G;
    gravity_true = gravityTorque(gen3, q);
    
    all_gravity_est(config_idx, :) = gravity_est';
    all_gravity_true(config_idx, :) = gravity_true';
    
    %% Joint Accelerations (Forward Dynamics)
    qddot_matlab = forwardDynamics(gen3, q, qdot);
    qddot_mine = myKG3.M \ (-myKG3.C*qdot - myKG3.G);
    
    all_qddots_matlab(config_idx, :) = qddot_matlab';
    all_qddots_mine(config_idx, :) = qddot_mine';
end

%% Figure 1: Mass Matrix Errors
figure(1);
sgtitle('Mass Matrix Comparison', 'FontSize', 16, 'FontWeight', 'bold');

plot(1:numConfigurations, frobenius_errors, 'bo-', 'LineWidth', 2, 'MarkerSize', 8);
xlabel('Configuration Index');
ylabel('Frobenius Error');


fprintf('Mean Frobenius Error: %.6f\n', mean(frobenius_errors));
fprintf('Max Frobenius Error: %.6f\n', max(frobenius_errors));

%% Figure 2: Coriolis Forces Comparison
figure(2);
sgtitle('Coriolis/Centrifugal Forces Comparison', 'FontSize', 16, 'FontWeight', 'bold');

for joint = 1:7
    subplot(3, 3, joint);
    hold on;
    
    plot(1:numConfigurations, all_coriolis_true(:, joint), 'bo-', ...
         'LineWidth', 2, 'MarkerSize', 6, 'DisplayName', 'MATLAB');
    plot(1:numConfigurations, all_coriolis_est(:, joint), 'r^--', ...
         'LineWidth', 2, 'MarkerSize', 6, 'DisplayName', 'Derived');
    
    xlabel('Configuration Index');
    ylabel('Coriolis Torque (Nm)');
    title(sprintf('Joint %d Coriolis Forces', joint), 'FontWeight', 'bold');
    legend('Location', 'best');
    
end

% Error comparison in subplot 8
subplot(3, 3, 8);
coriolis_errors = abs(all_coriolis_true - all_coriolis_est);
boxplot(coriolis_errors, 'Labels', {'J1','J2','J3','J4','J5','J6','J7'});
ylabel('Absolute Error (Nm)');
title('Coriolis Error Distribution by Joint', 'FontWeight', 'bold');

% Summary statistics in subplot 9
subplot(3, 3, 9);
axis off;
overall_rms = sqrt(mean(mean((all_coriolis_true - all_coriolis_est).^2)));
max_error = max(max(abs(all_coriolis_true - all_coriolis_est)));
text(0.1, 0.9, 'Coriolis Summary:', 'FontSize', 12, 'FontWeight', 'bold');
text(0.1, 0.7, sprintf('Overall RMS Error: %.4f Nm', overall_rms), 'FontSize', 10);
text(0.1, 0.5, sprintf('Maximum Error: %.4f Nm', max_error), 'FontSize', 10);
text(0.1, 0.3, sprintf('Mean Abs Error: %.4f Nm', mean(mean(abs(all_coriolis_true - all_coriolis_est)))), 'FontSize', 10);

%% Figure 3: Gravity Torques Comparison
figure(3);
sgtitle('Gravity Torques Comparison', 'FontSize', 16, 'FontWeight', 'bold');

for joint = 1:7
    subplot(3, 3, joint);
    hold on;
    
    plot(1:numConfigurations, all_gravity_true(:, joint), 'bo-', ...
         'LineWidth', 2, 'MarkerSize', 6, 'DisplayName', 'MATLAB');
    plot(1:numConfigurations, all_gravity_est(:, joint), 'r^--', ...
         'LineWidth', 2, 'MarkerSize', 6, 'DisplayName', 'Derived');
    
    xlabel('Configuration Index');
    ylabel('Gravity Torque (Nm)');
    title(sprintf('Joint %d Gravity Torque', joint), 'FontWeight', 'bold');
    legend('Location', 'best');
    
end

% Error comparison in subplot 8
subplot(3, 3, 8);
gravity_errors = abs(all_gravity_true - all_gravity_est);
boxplot(gravity_errors, 'Labels', {'J1','J2','J3','J4','J5','J6','J7'});
ylabel('Absolute Error (Nm)');
title('Gravity Error Distribution by Joint', 'FontWeight', 'bold');
grid on;

% Summary statistics in subplot 9
subplot(3, 3, 9);
axis off;
overall_rms = sqrt(mean(mean((all_gravity_true - all_gravity_est).^2)));
max_error = max(max(abs(all_gravity_true - all_gravity_est)));
text(0.1, 0.9, 'Gravity Summary:', 'FontSize', 12, 'FontWeight', 'bold');
text(0.1, 0.7, sprintf('Overall RMS Error: %.4f Nm', overall_rms), 'FontSize', 10);
text(0.1, 0.5, sprintf('Maximum Error: %.4f Nm', max_error), 'FontSize', 10);
text(0.1, 0.3, sprintf('Mean Abs Error: %.4f Nm', mean(mean(abs(all_gravity_true - all_gravity_est)))), 'FontSize', 10);

%% Figure 4: Joint Accelerations Comparison
figure(4);
sgtitle('Joint Accelerations Comparison', 'FontSize', 16, 'FontWeight', 'bold');

for joint = 1:7
    subplot(3, 3, joint);
    hold on;
    
    plot(1:numConfigurations, all_qddots_matlab(:, joint), 'bo-', ...
         'LineWidth', 2, 'MarkerSize', 6, 'DisplayName', 'MATLAB');
    plot(1:numConfigurations, all_qddots_mine(:, joint), 'r^--', ...
         'LineWidth', 2, 'MarkerSize', 6, 'DisplayName', 'Derived');
    
    xlabel('Configuration Index');
    ylabel('Acceleration (rad/s²)');
    title(sprintf('Joint %d Acceleration', joint), 'FontWeight', 'bold');
    legend('Location', 'best');
    
end

% Error comparison in subplot 8
subplot(3, 3, 8);
acceleration_errors = abs(all_qddots_matlab - all_qddots_mine);
boxplot(acceleration_errors, 'Labels', {'J1','J2','J3','J4','J5','J6','J7'});
ylabel('Absolute Error (rad/s²)');
title('Acceleration Error Distribution by Joint', 'FontWeight', 'bold');

% Summary statistics in subplot 9
subplot(3, 3, 9);
axis off;
overall_rms = sqrt(mean(mean((all_qddots_matlab - all_qddots_mine).^2)));
max_error = max(max(abs(all_qddots_matlab - all_qddots_mine)));
text(0.1, 0.9, 'Acceleration Summary:', 'FontSize', 12, 'FontWeight', 'bold');
text(0.1, 0.7, sprintf('Overall RMS Error: %.4f rad/s²', overall_rms), 'FontSize', 10);
text(0.1, 0.5, sprintf('Maximum Error: %.4f rad/s²', max_error), 'FontSize', 10);
text(0.1, 0.3, sprintf('Mean Abs Error: %.4f rad/s²', mean(mean(abs(all_qddots_matlab - all_qddots_mine)))), 'FontSize', 10);

%% Summary Statistics
fprintf('\n=== SUMMARY STATISTICS ===\n');
fprintf('Frobenius Error Statistics:\n');
fprintf('  Mean: %.6f, Std: %.6f, Max: %.6f\n', ...
        mean(frobenius_errors), std(frobenius_errors), max(frobenius_errors));

fprintf('\nCoriolis Force RMS Errors by Joint:\n');
for joint = 1:7
    rms_error = sqrt(mean((all_coriolis_true(:, joint) - all_coriolis_est(:, joint)).^2));
    fprintf('  Joint %d: %.6f Nm\n', joint, rms_error);
end

fprintf('\nGravity Torque RMS Errors by Joint:\n');
for joint = 1:7
    rms_error = sqrt(mean((all_gravity_true(:, joint) - all_gravity_est(:, joint)).^2));
    fprintf('  Joint %d: %.6f Nm\n', joint, rms_error);
end

fprintf('\nJoint Acceleration RMS Errors by Joint:\n');
for joint = 1:7
    rms_error = sqrt(mean((all_qddots_matlab(:, joint) - all_qddots_mine(:, joint)).^2));
    fprintf('  Joint %d: %.6f rad/s²\n', joint, rms_error);
end

%% Additional detailed comparison for worst case
[~, worst_idx] = max(frobenius_errors);
fprintf('\n=== WORST CASE ANALYSIS (Configuration %d) ===\n', worst_idx);
fprintf('Configuration: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', all_configs(worst_idx, :));
fprintf('Frobenius Error: %.6f\n', frobenius_errors(worst_idx));

function qdot = generateRandomQdot(method, varargin)

    % Generates random joint velocities for the 7-DOF KG3 robot
    % 
    % Usage:
    %   qdot = generateRandomQdot('uniform')           % Uniform [-1, 1] rad/s
    %   qdot = generateRandomQdot('uniform', max_vel)  % Uniform [-max_vel, max_vel]
    %   qdot = generateRandomQdot('gaussian')          % Gaussian with std=0.5
    %   qdot = generateRandomQdot('gaussian', sigma)   % Gaussian with std=sigma
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