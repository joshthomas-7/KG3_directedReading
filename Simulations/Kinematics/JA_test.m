%%% Joshua Thomas
%%% C3376353

clear
close all
clc


%% Loading the KG3 snd placing it in a random configuration
gen3 = loadrobot("kinovaGen3", "DataFormat", "column", "Version", 2);
showdetails(gen3)

% Placing the KG3 in a random configuration
q_config = randomConfiguration(gen3);
transform = getTransform(gen3, q_config, 'end_effector_link');

%% Computing the geometric Jacobian 

% Using the prebuilt model (this Jacobian as the orientation and position
% jacobians swapped
J_prebuilt = geometricJacobian(gen3, q_config, 'end_effector_link');
disp('MATLAB J = ')
disp(J_prebuilt)

% Computing the analytical Jacobian using the derived FKM
[~,Ts] = KG3_FKM();
[r08, R, T_all] = KG3_FKM_simple(q_config, Ts);
J = KG3_JGEO(T_all);
disp('Derived J = ')
disp(J)

% Extracting the euler angles to compute the analytical jacobian
eul = rotm2eul(R);
phi = eul(1);
theta = eul(2);

JA = KG3_JA(phi,theta,J);
disp('Analytical Jacobian')
disp(JA)
