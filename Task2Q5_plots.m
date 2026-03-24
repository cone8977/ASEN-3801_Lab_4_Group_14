clc; clear; close all;



% Task 2 Question 5

function var_dot = QuadrotorEOMwithRateFeedback (t, var, g, m, I, nu, mu)


% From function QuadrotorEOM but Fc and Gc from 2.3 instead of having km
% and D and computing the motor forces and all

[Fc, Gc] = RotationDerivativeFeedback(var, m, g)

State.x=var(1);
State.y=var(2);
State.z=var(3);
State.phi=var(4);
State.theta=var(5);
State.psi=var(6);
State.u=var(7);
State.v=var(8);
State.w=var(9);
State.p=var(10);
State.q=var(11);
State.r=var(12);
%% Trig structure 
% Psi
Trig.cpsi=cos(State.psi);
Trig.spsi=sin(State.psi);
Trig.tpsi=tan(State.psi);
% Theta
Trig.ctheta=cos(State.theta);
Trig.stheta=sin(State.theta);
Trig.ttheta=tan(State.theta);
% Phi
Trig.cphi=cos(State.phi);
Trig.sphi=sin(State.phi);
Trig.tphi=tan(State.phi);

%% Inetria Struct
In.x=I(1,1);
In.y=I(2,2);
In.z=I(3,3);


%This is the change done relative to the quadrotorEOM, the Zc, Lc, Mc, Nc
%are computed using Fc and Gc rather than using motor forces and all (as
%you can see below this chunk is the commented section of the original
%quadrotorEOM.

Control.Z = Fc(3)
Control.L = Gc(1)
Control.M = Gc(2)
Control.N = Gc(3)



% %% Control Struct
% control=[-1 -1 -1 -1; -d./sqrt(2) -d./sqrt(2) d./sqrt(2) d./sqrt(2); d./sqrt(2) -d./sqrt(2) -d./sqrt(2) d./sqrt(2); km -km km -km].*motor_forces;
% Control.Z=sum(control(1,:));
% Control.L=sum(control(2,:));
% Control.M=sum(control(3,:));
% Control.N=sum(control(4,:));



%% Aero Struct
Force= -nu.*norm([State.u State.v State.w]).*[State.u; State.v; State.w];
Moments=-mu.*norm([State.p State.q State.r]).*[State.p; State.q; State.r];
Aero.X=Force(1);
Aero.Y=Force(2);
Aero.Z=Force(3);
Aero.L=Moments(1);
Aero.M=Moments(2);
Aero.N=Moments(3);

%% X_dot, Y_dot, Z_dot
Pos_dot=[Trig.ctheta.*Trig.cpsi, (Trig.sphi.*Trig.stheta.*Trig.cpsi)-(Trig.cphi.*Trig.spsi), (Trig.cphi.*Trig.stheta.*Trig.cpsi)-(Trig.sphi.*Trig.spsi); ...
                     Trig.ctheta.*Trig.spsi, (Trig.sphi.*Trig.stheta.*Trig.spsi)+(Trig.cphi.*Trig.cpsi), (Trig.cphi.*Trig.stheta.*Trig.spsi)-(Trig.sphi.*Trig.cpsi); ...
                     -Trig.stheta, Trig.ctheta.*Trig.sphi, Trig.ctheta.*Trig.cphi] * [State.u;State.v;State.w];

X_dot = Pos_dot(1);
Y_dot = Pos_dot(2);
Z_dot = Pos_dot(3);


%% Phi_dot, Theta_dot, Psi_dot
Angle_dot=[ 1, Trig.sphi.*Trig.ttheta, Trig.cphi.*Trig.ttheta;
            0, Trig.cphi, -Trig.sphi
            0, Trig.sphi./Trig.ctheta, Trig.cphi./Trig.ctheta]*[State.p;State.q;State.r;]; 


Phi_dot = Angle_dot(1);
Theta_dot = Angle_dot(2);
Psi_dot = Angle_dot(3);


%% U_dot, V_dot, W_dot

v_dot= cross([State.u,State.v,State.w],[State.p,State.q,State.r])' + g.*[-Trig.stheta;Trig.ctheta.*Trig.sphi ;Trig.ctheta.*Trig.cphi] +[Aero.X; Aero.Y; Aero.Z]./m+[0;0;Control.Z]./m;

U_dot = v_dot(1);
V_dot = v_dot(2);
W_dot = v_dot(3);


%% P_dot, Q_dot, R_dot
Omega_dot= [ ((In.y-In.z)./In.x).*State.q.*State.r;((In.z-In.x)./In.y).*State.p.*State.r;((In.x-In.y)./In.z).*State.q.*State.p]+[(1/In.x).*Aero.L; (1/In.y).*Aero.M; (1/In.z).*Aero.N]+[(1/In.x).*Control.L; (1/In.y).*Control.M; (1/In.z).*Control.N];

P_dot = Omega_dot(1);
Q_dot = Omega_dot(2);
R_dot = Omega_dot(3);

% %% Ground Collison
% if(State.z>=0)        
%     Z_dot=0; 
%     U_dot=0;
%     V_dot=0;
%     W_dot=0;
%     P_dot=0;
%     Q_dot=0;
%     R_dot=0;
% end

%% Compilation
var_dot=[ X_dot; Y_dot; Z_dot; Phi_dot; Theta_dot; Psi_dot; U_dot; V_dot; W_dot; P_dot; Q_dot; R_dot];


end





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
[time,aircraft_state_array] = ode45(@(t,var) QuadrotorEOMwithRateFeedback(t, var, g, m, I, nu, mu),t,var);




[Fc, Gc] = RotationDerivativeFeedback(var, m, g)

control = [Fc(3); Gc(1); Gc(2); Gc(3)]

% % Calculate Control Forces
% control = [  -1,          -1,         -1,         -1      ; % Set Up Matrix 
%           -d./sqrt(2), -d./sqrt(2), d./sqrt(2), d./sqrt(2); 
%           d./sqrt(2), -d./sqrt(2), -d./sqrt(2), d./sqrt(2); 
%               km,         -km,          km,        -km    ].* motor_forces;





% Map Control Force Values over Time Interval
control_input_array(:,1) = repmat(sum(control(1,:)),length(time),1);
control_input_array(:,2) = repmat(sum(control(2,:)),length(time),1);
control_input_array(:,3) = repmat(sum(control(3,:)),length(time),1);
control_input_array(:,4) = repmat(sum(control(4,:)),length(time),1);

fig = 1:6; % Set figure numbers
col = ['r';'r';'r';'b';'b';'b';'k']; % Set colors for plotting

% Plot Non-linearized Simulation
PlotAircraftSim(time,aircraft_state_array,control_input_array,fig,col)
