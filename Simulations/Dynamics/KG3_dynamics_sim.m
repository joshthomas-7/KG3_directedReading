%%% C3376353
%%% Joshua Thomas
clear
close all
clc

%% Loading the KG3 
gen3 = loadrobot("kinovaGen3", "DataFormat", "column", "Version", 2, Gravity=[0 0 -9.81]);
% show(gen3);
showdetails(gen3)

% Getting the home configuration
q = homeConfiguration(gen3);

%% Validating the mass matrix calculations
M = massMatrix(gen3)

% Need all transformation matricies to compute the geometric jacobian

% Initialising the KG3 class
myKG3 = KG3();



%% Computing the joint accelerations in the home configuration 
% Using the matlab model
qddot_mat = forwardDynamics(gen3)

% Using the derived model