%%% Joshua Thomas
%%% C3376353

clear
close all
clc


%% Loading the KG3 snd placing it in a random configuration
gen3 = loadrobot("kinovaGen3", "DataFormat", "column", "Version", 2);
showdetails(gen3)

% Placing the KG3 in a random configuration
q_config = homeConfiguration(gen3);
transform = getTransform(gen3, q_config, 'end_effector_link');

%% Computing the geometric Jacobian 

% Using the prebuilt model
J_prebuilt = geometricJacobian(gen3, q_config, 'end_effector_link')

[~,Ts] = KG3_FKM();
[r08, R, T_all] = KG3_FKM_simple(q_config, Ts); %T_all verified to be correct

J = KG3_GEOJ(T_all);
% J_toMatchMatlab = [J(4:end);J(1:3)]