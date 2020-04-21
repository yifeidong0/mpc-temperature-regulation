% Init
clear all
close all
addpath(genpath(cd));
load('system/parameters_scenarios.mat');

%% E.g. execute simulation with LQR
% clear persisten variables of function controller_lqr
clear controller_lqr; 

T0_1=[-18;1.3;7.32];
T0_2=[-22;0;2.82];
% execute simulation starting from T0_1 using lqr controller with scenario 1
[T, p] = simulate_truck(T0_2, @controller_lqr, scen1);