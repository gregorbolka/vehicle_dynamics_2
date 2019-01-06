
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main Vehicle Simulation %
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% This is the main file for the simulation of a vehicle system. 
% In this file all parameters and inputs are defined, the integration
% algorithm is called and the results are post-processed. 

clc
close all
clear all

%% Define vehicle parameters:
vehicle_param = set_vehicle_param;


%% Define simulation inputs:
inputs = set_inputs(vehicle_param);


%% Perform time-integration of vehicle system over inputs:
[X, data] = time_integration(vehicle_param, inputs);


%% Post-process results: 
post_processing(X, data, vehicle_param, inputs);

% %% Draw the linear tyre model
% Kyf = vehicle_param.Cy_f;
% g = 9.81;
% Fzrl = data(6,1);
% ay = linspace(-0.5,0.5, length(data(1,:)));
% alpha_rl_l = Fzrl.*ay/Kyf/g;
% 
% figure 
% hold on
% grid on
% xlim([-0.5,0.5]);
% title('Linear tyre');
% xlabel('lateral acceleration m/s^2', 'FontSize', 14);
% ylabel('slip angle of left rear wheel', 'FontSize', 14);
% plot(ay, alpha_rl_l);

% figure
% hold on
% grid on
% % xlim([-0.5,0.5]);
% % ylim([-0.06,0.06]);
% title('Dugoff model');
% xlabel('lateral acceleration m/s^2', 'FontSize', 14);
% ylabel('slip angle of wheels', 'FontSize', 14);
% plot(data(1,5:end), data(2:5,5:end),'LineWidth', 2);
% legend('Front Left', 'Front Right', 'Rear Left', 'Rear Right');