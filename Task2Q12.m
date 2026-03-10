clc; clear; close all;

%% Constants/Varibales 

g = 9.81; % Acceleration due to gravity (m/s^2)
m = 0.068; % QR mass (kg)
I = [5.8*10^(-5),0,0;
     0,7.2*10^(-5),0;
     0,0,1.0*10^(-4)]; % QR intertia matrix 
d = 0.06; % Radial distance to cg
km = 0.0024; % Moment coefficient 
nu = 1*10^(-3);
mu = 2*10^(-6);

t = [0, 10]; % Simulate for 0-10 seconds
var=[0;0;0;         % Inertial Position [x,y,z]
     0;0;0;         % Inertial Attitude
     0;0;0;         % Body Velocities
     0;0;0.1];      % Body Angular Acceleration
motor_forces = [m*g/4; m*g/4; m*g/4; m*g/4]; % For hover

%% Non-Linearized 

% Run Simulation
[time,aircraft_state_array] = ode45(@(t,var) QuadrotorEOM(t, var, g, m,...
                                    I, d, km, nu, mu, motor_forces),t,var);

% Calculate Control Forces
control = [  -1,          -1,         -1,         -1      ; % Set Up Matrix 
          -d./sqrt(2), -d./sqrt(2), d./sqrt(2), d./sqrt(2); 
          d./sqrt(2), -d./sqrt(2), -d./sqrt(2), d./sqrt(2); 
              km,         -km,          km,        -km    ].* motor_forces;

% Map Control Force Values over Time Interval
control_input_array(:,1) = repmat(sum(control(1,:)),length(time),1);
control_input_array(:,2) = repmat(sum(control(2,:)),length(time),1);
control_input_array(:,3) = repmat(sum(control(3,:)),length(time),1);
control_input_array(:,4) = repmat(sum(control(4,:)),length(time),1);

fig = 1:6; % Set figure numbers
col = ['r';'r';'r';'b';'b';'b';'k']; % Set colors for plotting

% Plot Non-linearized Simulation
PlotAircraftSim(time,aircraft_state_array,control_input_array,fig,col)

%% Lienarized

% Set Perturbations at Hover
deltaFc = 0; deltaGc = [0,0,0];

% Run Simulation
[time,aircraft_state_array] = ode45(@(t,var) QuadrotorEOM_Linearized(t,...
                                    var, g, m, I, deltaFc, deltaGc),t,var);

col = ['m--';'m--';'m--';'c--';'c--';'c--';'k--']; % Set new colors

% Plot Linearized Case
PlotAircraftSim(time,aircraft_state_array,control_input_array,fig,col)
