%%% Joshua Thomas
%%% C3376353

clear
close all
clc

Ndof = 7;
rho = 0.1;

%% Setting up the KG3 direct kinematic model
[transform_computed, Ts, Rs, As] = KG3_FKM;

% Extracting the position vector
f = transform_computed(1:end-1,end);

%% Constructing the IKM optimisation problem
% Uses sigma = [q s];
W = eye(Ndof); % joint weighting matrix
H = [W zeros(Ndof);
     zeros(Ndof) zeros(Ndof)];
f = [zeros(1,Ndof) rho*ones(1,Ndof)]


%% Setting up the robot model
% Load the KG3 robot 
gen3 = loadrobot("kinovaGen3", "DataFormat", "column");
% show(gen3);
showdetails(gen3)