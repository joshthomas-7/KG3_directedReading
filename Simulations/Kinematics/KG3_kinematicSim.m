%%% Joshua Thomas
%%% C3376353

clear
close all
clc

%% Setting up the KG3 direct kinematic model
[transform_computed, Ts, Rs, As] = KG3_FKM;


%% Setting up the robot model
% Load the KG3 robot 
gen3 = loadrobot("kinovaGen3", "DataFormat", "column");
% show(gen3);
showdetails(gen3)


% Get the default joint configuration (home position)
% q_config = gen3.homeConfiguration


% % Place the KG3 in a random configuration
q_config = randomConfiguration(gen3);
show(gen3, q_config)
% 
% % Find the homogenous transform of link 7 relative to the base
transform = getTransform(gen3, q_config, "EndEffector_Link")

q1 = q_config(1);
q2 = q_config(2);
q3 = q_config(3);
q4 = q_config(4);
q5 = q_config(5);
q6 = q_config(6);
q7 = q_config(7);

transform_computed = vpa(subs(transform_computed),4)
