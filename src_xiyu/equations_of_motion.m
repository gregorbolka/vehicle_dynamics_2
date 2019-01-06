function    [dX, data] = equations_of_motion(vehicle_param, inputs, X, dX_old, k)
%
% This file solves the equations of motion with respect to the acceleration
% in order to determine dX. Other interesting variables can be stored in
% 'data'. 
%
%   dX = [velocity;
%         acceleration]
%          

% X = [x, y, phi, phifr, phifl, phirr, phirl, x_dot, y_dot, phi_dot, phifr_dot, phifl_dot, phirr_dot, phirl_dot]


dX = zeros(vehicle_param.n_dofs*2,1);

% Velocity equals the velocity from the previous time-step:
dX(1:vehicle_param.n_dofs) = X(vehicle_param.n_dofs+1:end);

%% Perform projections on velocities:
% X = [x, y, phi, phifr, phifl, phirr, phirl, x_dot, y_dot, phi_dot, phifr_dot, phifl_dot, phirr_dot, phirl_dot]
phi = X(3);
delta = inputs.delta(k);
% theta = inputs.inclin_angle;
Tfl = inputs.T(1,k);
Tfr = inputs.T(2,k);
Trl = inputs.T(3,k);
Trr = inputs.T(4,k);
a = vehicle_param.a;
b = vehicle_param.b;
Iw = vehicle_param.I_w;
Izz = vehicle_param.Izz;
r = vehicle_param.r_w;
t = vehicle_param.Df;
m = vehicle_param.M;
%% Calculate tyre forces:
[Ffl, Ffr, Frl, Frr, F_aero,alpha] = get_tyre_forces(vehicle_param, inputs, X, dX_old, k);
Fxfl = Ffl(1);
Fyfl = Ffl(2);
Fxfr = Ffr(1);
Fyfr = Ffr(2);
Fxrl = Frl(1);
Fyrl = Frl(2);
Fxrr = Frr(1);
Fyrr = Frr(2);
%% Determine accelerations:
% IN GLOBAL
% X = [x, y, phi, phifr, phifl, phirr, phirl, x_dot, y_dot, phi_dot, phifr_dot, phifl_dot, phirr_dot, phirl_dot]
% x
Fxg = (Fxfl+Fxfr)*cos(phi+delta) - (Fyfl+Fyfr)*sin(phi+delta) + (Fxrl+Fxrr)*cos(phi) - (Fyrl+Fyrr)*sin(phi) - F_aero*cos(phi);
dX(vehicle_param.n_dofs + 1) = Fxg/m;
% y
Fyg = (Fxfl+Fxfr)*sin(phi+delta) + (Fyfl+Fyfr)*cos(phi+delta) + (Fxrl+Fxrr)*sin(phi) + (Fyrl+Fyrr)*cos(phi) - F_aero*sin(phi);
dX(vehicle_param.n_dofs + 2) = Fyg/m;
% yaw
moment = (-(Fxfl*cos(delta)-Fyfl*sin(delta))+ (Fxfr*cos(delta)-Fyfr*sin(delta)) - Fxrl + Fxrr )*t/2 ...
                    + (Fxfl*sin(delta)+Fyfl*cos(delta)+Fyfr*cos(delta)+Fxfr*sin(delta))*a + (-Fyrl - Fyrr)*b;
dX(vehicle_param.n_dofs + 3) = moment/Izz;
% wheels
dX(vehicle_param.n_dofs + 4) = (Tfr - r*Fxfr)/Iw;
dX(vehicle_param.n_dofs + 5) = (Tfl - r*Fxfl)/Iw;
dX(vehicle_param.n_dofs + 6) = (Trr - r*Fxrr)/Iw;
dX(vehicle_param.n_dofs + 7) = (Trl - r*Fxrl)/Iw;
% Additional interesting data can be stored in data:
% lateral in car frame
a_lat = -dX(vehicle_param.n_dofs + 1)*sin(phi) + dX(vehicle_param.n_dofs + 2)*cos(phi);
data = [a_lat; alpha';Frl(3)];


