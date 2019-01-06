function inputs = set_inputs(vehicle_param)
%
% This function is for defining the inputs for the time-simulation in a structure
% 'inputs'. This always has to include the time-vector and the time-step.  
%

% Obligatory time information:
inputs.dt = 0.001;
inputs.time = (0:inputs.dt:6);
n = length(inputs.time);

% Initial velocity
inputs.v0 = 15;

% Other inputs are dependent on the model used. (eg. steering angle, engine
% torque, ...)

% Wheel Torque:
inputs.T = [ones(1,n)*0; ones(1,n)*0; ones(1,n)*100; ones(1,n)*100];

% Inclination of road:
inputs.inclin_angle = 0*pi/180; % [rad]

% Steering angle:
%inputs.delta = 5*ones(size(inputs.time))*pi/180*0;
inputs.delta = GetInputSignal(inputs.dt);

