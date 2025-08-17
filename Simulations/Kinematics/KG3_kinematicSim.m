%% Joshua Thomas
%% C3376353

clear
close all
clc

%% Setting up the KG3 direct kinematic model
[transform_computed, Ts, Rs, As] = KG3_FKM();


%% Setting up the robot model
% Load the KG3 robot 
% gen3 = loadrobot("kinovaGen3", "DataFormat", "column", "Version", 1);
gen3 = loadrobot("kinovaGen3", "DataFormat", "column", "Version", 2);
% show(gen3);
showdetails(gen3)


% Get the default joint configuration (home position)
% q_config = gen3.homeConfiguration

Niter = 100;
pose_des_arr = zeros(7, Niter);
pose_comp_arr = zeros(7, Niter);

% Initialize variables to store maximum errors
max_position_error = 0;
max_orientation_error = 0;

Ts = KG3_preCompute();
for i = 1:Niter
    %% Place the KG3 in a random configuration
    q_config = randomConfiguration(gen3);
    % show(gen3, q_config)

    %% Find the homogenous transform
    transform = getTransform(gen3, q_config, "end_effector_link");
    p_des = transform(1:end-1, end);
    R_desired = transform(1:3, 1:3);
    q_des = rotm2quat(R_desired);

    % Compute the pose using forward kinematics
    [p_comp, R_comp] = KG3_FKM_simple(q_config, Ts);
    q_comp = rotm2quat(R_comp);

    % Store the desired and computed poses
    pose_des = [p_des; q_des.'];
    pose_comp = [p_comp; q_comp.'];

    pose_des_arr(:, i) = pose_des;
    pose_comp_arr(:, i) = pose_comp;

    % Calculate position error (Euclidean distance)
    position_error = norm(p_des - p_comp);

    % Calculate orientation error (using quaternion)
    cosTheta = dot(q_des, q_comp);
    orientation_error = acosd(2 * (cosTheta^2) - 1); % in degrees

    % Update maximum errors
    max_position_error = max(max_position_error, position_error);
    max_orientation_error = max(max_orientation_error, orientation_error);
end

% Print the worst-case errors
fprintf('Worst Case Position Error: %.4f\n', max_position_error);
fprintf('Worst Case Orientation Error: %.4f degrees\n', max_orientation_error);

%% Plotting
configurations = linspace(1,Niter,Niter);

figure(1)
subplot(3,1,1)
hold on
title('End Effector Position Comparison', 'FontSize', 20)
plot(configurations, pose_des_arr(1,:), 'LineWidth', 3)
plot(configurations, pose_comp_arr(1,:), 'LineWidth', 3, 'LineStyle','--')
ylabel('x Position (m)', 'FontSize', 18)
legend('MATLAB', 'Derived', 'FontSize', 16)

subplot(3,1,2)
hold on
plot(configurations, pose_des_arr(2,:), 'LineWidth', 3)
plot(configurations, pose_comp_arr(2,:), 'LineWidth', 3, 'LineStyle','--')
ylabel('y Position (m)', 'FontSize', 18)
% legend('True', 'Computed', 'FontSize', 16)

subplot(3,1,3)
hold on
plot(configurations, pose_des_arr(3,:), 'LineWidth', 3)
plot(configurations, pose_comp_arr(3,:), 'LineWidth', 3, 'LineStyle','--')
ylabel('z Position (m)', 'FontSize', 18)
xlabel('Configuration Number', 'FontSize', 18)
% legend('True', 'Computed', 'FontSize', 16)


figure(2)
subplot(4,1,1)
hold on
title('End Effector Orientation Comparison', 'FontSize', 20)
plot(configurations, pose_des_arr(4,:), 'LineWidth', 3)
plot(configurations, pose_comp_arr(4,:), 'LineWidth', 3, 'LineStyle','--')
ylabel('\eta', 'FontSize', 18)
legend('MATLAB', 'Derived', 'FontSize', 16)

subplot(4,1,2)
hold on
plot(configurations, pose_des_arr(5,:), 'LineWidth', 3)
plot(configurations, pose_comp_arr(5,:), 'LineWidth', 3, 'LineStyle','--')
ylabel('\epsilon_1', 'FontSize', 18)
% legend('True', 'Computed', 'FontSize', 16)

subplot(4,1,3)
hold on
plot(configurations, pose_des_arr(6,:), 'LineWidth', 3)
plot(configurations, pose_comp_arr(6,:), 'LineWidth', 3, 'LineStyle','--')
ylabel('\epsilon_2', 'FontSize', 18)
% xlabel('Configuration Number', 'FontSize', 18)

subplot(4,1,4)
hold on
plot(configurations, pose_des_arr(7,:), 'LineWidth', 3)
plot(configurations, pose_comp_arr(7,:), 'LineWidth', 3, 'LineStyle','--')
ylabel('\epsilon_3', 'FontSize', 18)
xlabel('Configuration Number', 'FontSize', 18)
% legend('True', 'Computed', 'FontSize', 16)


% %% Place the KG3 in a random configuration
% q_config = randomConfiguration(gen3);
% show(gen3, q_config)
% % 
% %% Find the homogenous transform
% % transform = getTransform(gen3, q_config, "EndEffector_Link")
% transform = getTransform(gen3, q_config, "end_effector_link")
% p_des = transform(1:end-1,end);
% R_desired = transform(1:3,1:3);
% q_des = rotm2quat(R_desired);
% 
% 
% q1 = q_config(1);
% q2 = q_config(2);
% q3 = q_config(3);
% q4 = q_config(4);
% q5 = q_config(5);
% q6 = q_config(6);
% q7 = q_config(7);
% 
% % Using the joint positions to compute the expected transform
% transform_computed = vpa(subs(transform_computed),4)
% % Extracting the pose from the homogenous transform matricies
% p_comp = double(transform_computed(1:end-1,end));
% R_comp = double(transform_computed(1:3,1:3));
% q_comp = rotm2quat(R_comp);
% 
% % Computing the desired pose and the computed pose
% pose_des = [p_des;q_des.'];
% pose_comp = [p_comp;q_comp.'];
% 
% disp('True Pose & Computed Pose')
% disp([pose_des pose_comp])
% 
% % Finding the error between the poses
% pose_err = pose_des - pose_comp
