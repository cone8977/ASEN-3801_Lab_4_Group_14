% Contributors: Victor Turpin, Liz Thompson
% Course number: ASEN 3801
% File name: QuadrotorEOM
% Created: 3/10/26

function var_dot = QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs:   time = simulation time
%           var = 12x1 state vector [x,y,z,phi,theta,psi,u,v,w,p,q,r] 
%           g = acceleration due to gravity (m/s^2) 
%           m = QR mass (kg)
%           I = QR inertia matrix (km*m^2)
%           deltaFc = Linearized distrubance in QR force Zc
%           deltaGc = Linearized distrubance in QR moments (Lc,Mc,Nc)
% 
% Output:   var_dot: 12x1 derivative of the state vector
%
% Methodology: Using derivations from class, calculate the dotted state
% vaiables given values for the previous step and using linearized formulas
% and values. This function will be used by ode45 to simulate QR flight.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Assign Useful Names to Variables (x,y,z,phi,theta,etc...)

State.x=var(1); State.y=var(2); State.z=var(3);             % Position
State.phi=var(4); State.theta=var(5); State.psi=var(6);     % Attitude
State.u=var(7); State.v=var(8); State.w=var(9);             % Velocity
State.p=var(10); State.q=var(11); State.r=var(12);          % Angular Rate

%% Extracting Inertia Values

In.x = I(1,1); In.y = I(2,2); In.z = I(3,3);

%% Extracting Disturbance Values

% Force
Disturb.Z = deltaFc;

% Moments
Disturb.L = deltaGc(1); Disturb.M = deltaGc(2); Disturb.N = deltaGc(3);

%% X_dot, Y_dot, Z_dot

X_dot = State.u; Y_dot = State.v; Z_dot = State.w;

%% Phi_dot, Theta_dot, Psi_dot

Phi_dot = State.p; Theta_dot = State.q; Psi_dot = State.r;

%% U_dot, V_dot, W_dot

U_dot = -State.theta * g; V_dot = State.phi * g; W_dot = Disturb.Z / m;

%% P_dot, Q_dot, R_dot

P_dot = Disturb.L/In.x; Q_dot = Disturb.M/In.y; R_dot = Disturb.N/In.z;

%% Compilation

var_dot=[ X_dot; Y_dot; Z_dot; Phi_dot; Theta_dot; Psi_dot; U_dot; V_dot; W_dot; P_dot; Q_dot; R_dot];

end
