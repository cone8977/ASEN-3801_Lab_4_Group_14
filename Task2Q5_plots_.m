clc; clear; close all;

% Contributors : Saketh Guttapalli, Cody Newton, Victor Turpin , Liz Thompson
% Course number: ASEN 3801
% File name: Task2Q5_plots_.m
% Created : 3/24/26




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
     0.1;0;0];      % Body Angular Acceleration
motor_forces = [m*g/4; m*g/4; m*g/4; m*g/4]; % For hover






% task 2 question 4

function motor_forces = ComputeMotorForces (Fc, Gc, d, km)

%Fc is the force vector [Xc Yc Zc]'

% using the last equation in slide 9

D = [Fc(3) Gc(1) Gc(2) Gc(3)]' ;

%This is from the Control variables from the left hand side of slide 9,
%where Zc, Lc, Mc, Nc corresponds to the values in D respectively

A_cl = [-1 -1 -1 -1;
    -d/(2^(1/2)) -d/(2^(1/2)) d/(2^(1/2)) d/(2^(1/2));
    d/(2^(1/2)) -d/(2^(1/2)) -d/(2^(1/2)) d/(2^(1/2));
    km -km km -km];

%This is the 4x4 matrix in the equation

motor_forces = A_cl\D;
end









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



%% Compilation
var_dot=[ X_dot; Y_dot; Z_dot; Phi_dot; Theta_dot; Psi_dot; U_dot; V_dot; W_dot; P_dot; Q_dot; R_dot];


end




function [var_dot, Control] = QuadrotorEOM(t,var,g,m,I,d,km,nu,mu,motor_forces) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs:   time = simulation time
%           var = 12x1 state vector [x,y,z,phi,theta,psi,u,v,w,p,q,r] 
%           g = acceleration due to gravity (m/s^2)
%           m = QR mass (kg)
%           I = QR inertia matrix (km*m^2)
%           d = Radial distance from cg to propeller (m)
%           km = Control moment coefficient (N*m/(N))
%           nu = Aerodynamic force coefficient (N/(m/s)^2)
%           mu = Aerodynamic moment coefficient (N*m/(rad/s)^2)
%           Motor Forces = 4x1 array of motor forces [f1; f2; f3; f4]
% 
% Output:   var_dot: 12x1 derivative of the state vector
%
% Methodology: Using derivations from class, calculate the dotted state
% vaiables given values for the previous step and control forces and
% moments. This function will be used by ode45 to simulate the QR flight. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Assign Useful Names to Variables (x,y,z,phi,theta,etc...)

State.x=var(1); State.y=var(2); State.z=var(3);             % Position
State.phi=var(4); State.theta=var(5); State.psi=var(6);     % Attitude
State.u=var(7); State.v=var(8); State.w=var(9);             % Velocity
State.p=var(10); State.q=var(11); State.r=var(12);          % Angular Rate

%% Set Up Euler Angle Structure 

% Calculate Trig values of Psi
Trig.cpsi = cos(State.psi); Trig.spsi = sin(State.psi);
Trig.tpsi = tan(State.psi);

% Calculate Trig values of Theta
Trig.ctheta = cos(State.theta); Trig.stheta = sin(State.theta);
Trig.ttheta = tan(State.theta);

% Calculate Trig values of Phi
Trig.cphi = cos(State.phi); Trig.sphi = sin(State.phi);
Trig.tphi = tan(State.phi);

%% Extracting Inertia Values

In.x = I(1,1); In.y = I(2,2); In.z = I(3,3);

%% Calcaulte Control Force/Moments

% Calculate Control Forces/Moments
control = [  -1,          -1,         -1,         -1      ; % Set Up Matrix 
          -d./sqrt(2), -d./sqrt(2), d./sqrt(2), d./sqrt(2); 
          d./sqrt(2), -d./sqrt(2), -d./sqrt(2), d./sqrt(2); 
              km,         -km,          km,        -km    ].* motor_forces;

% Allocate Control Force/Moment Values
Control.Z=sum(control(1,:)); Control.L=sum(control(2,:));
Control.M=sum(control(3,:)); Control.N=sum(control(4,:));

%% Calcaulte Aerodynamic Force/Moments

% Calculate Aerodynamic Forces/Moments
Force = -nu.*norm([State.u State.v State.w]).*[State.u; State.v; State.w];
Moments = -mu.*norm([State.p State.q State.r]).*[State.p; State.q; State.r];

% Allocate Aerodynamic Force/Moment Values
Aero.X = Force(1); Aero.Y = Force(2); Aero.Z = Force(3);
Aero.L = Moments(1); Aero.M = Moments(2); Aero.N = Moments(3);

%% X_dot, Y_dot, Z_dot

% Calculate d/dt[Inertial Position]
Pos_dot=[Trig.ctheta.*Trig.cpsi,... % Calculate B to E DCM
    (Trig.sphi.*Trig.stheta.*Trig.cpsi)-(Trig.cphi.*Trig.spsi),...
    (Trig.cphi.*Trig.stheta.*Trig.cpsi)-(Trig.sphi.*Trig.spsi);...
    (Trig.ctheta.*Trig.spsi),...
    (Trig.sphi.*Trig.stheta.*Trig.spsi)+(Trig.cphi.*Trig.cpsi),...
    (Trig.cphi.*Trig.stheta.*Trig.spsi)-(Trig.sphi.*Trig.cpsi);...
    (-Trig.stheta),...
    (Trig.ctheta.*Trig.sphi),...
    (Trig.ctheta.*Trig.cphi)] * [State.u;State.v;State.w];

% Allocate d/dt[Inertial Position]
X_dot = Pos_dot(1);
Y_dot = Pos_dot(2);
Z_dot = Pos_dot(3);


%% Phi_dot, Theta_dot, Psi_dot

% Calculate d/dt[Attiude]
Angle_dot = [1, Trig.sphi.*Trig.ttheta, Trig.cphi.*Trig.ttheta;
             0,        Trig.cphi,             -Trig.sphi      ;
             0, Trig.sphi./Trig.ctheta, Trig.cphi./Trig.ctheta]...
            *[State.p;State.q;State.r;]; 


% Allocate d/dt[Attiude]
Phi_dot = Angle_dot(1); Theta_dot = Angle_dot(2); Psi_dot = Angle_dot(3);

%% U_dot, V_dot, W_dot

% Calcualte d/dt[Velocity]
v_dot = cross([State.u,State.v,State.w],[State.p,State.q,State.r])' + g.*[-Trig.stheta;Trig.ctheta.*Trig.sphi ;Trig.ctheta.*Trig.cphi] +[Aero.X; Aero.Y; Aero.Z]./m+[0;0;Control.Z]./m;

% ALlocate d/dt[Velocity]
U_dot = v_dot(1); V_dot = v_dot(2); W_dot = v_dot(3);


%% P_dot, Q_dot, R_dot

% Calculate d/dt[Angular Rate]
Omega_dot = [ ((In.y-In.z)./In.x).*State.q.*State.r ;((In.z-In.x)./In.y).*State.p.*State.r ;((In.x-In.y)./In.z).*State.q.*State.p ]+[ (1/In.x).*Aero.L ;(1/In.y).*Aero.M ;(1/In.z).*Aero.N ] +[ (1/In.x).*Control.L ;(1/In.y).*Control.M ;(1/In.z).*Control.N];

% Allocate d/dt[Angular Rate]
P_dot = Omega_dot(1); Q_dot = Omega_dot(2); R_dot = Omega_dot(3);


%% Compile All Dotted States

var_dot=[ X_dot; Y_dot; Z_dot; Phi_dot; Theta_dot; Psi_dot; U_dot; V_dot; W_dot; P_dot; Q_dot; R_dot];

end








%% Non-Linearized with feedback

% Run Simulation
[times,aircraft_state_array] = ode45(@(t,var) QuadrotorEOMwithRateFeedback(t, var, g, m, I, nu, mu),t,var);


for i = 1: length(times)
    [Fc, Gc] = RotationDerivativeFeedback(aircraft_state_array(i,:), m, g)
    control = [Fc(3); Gc(1); Gc(2); Gc(3)]

control_input_array(i,1) = control(1);
control_input_array(i,2) = control(2);
control_input_array(i,3) = control(3);
control_input_array(i,4) = control(4);




Motor_Force = ComputeMotorForces (Fc, Gc, d, km)


Motor_Force_Array(i,1) = Motor_Force(1);
Motor_Force_Array(i,2) = Motor_Force(2);
Motor_Force_Array(i,3) = Motor_Force(3);
Motor_Force_Array(i,4) = Motor_Force(4);


end


fig = 1:6; % Set figure numbers
col = ['r';'r';'r';'b';'b';'b';'k']; % Set colors for plotting

% Plot Non-linearized Simulation
PlotAircraftSim(times,aircraft_state_array,control_input_array,fig,col, '25')





%% Plotting Control Inputs

figure (7); % Figure 7 for motor force
sgtitle('Motor Forces')
subplot(4,1,1); plot(times,Motor_Force_Array(:,1),col(3,:),LineWidth=1); grid on; hold on;
ylabel('F1'); xlim([0,max(times)]);
subplot(4,1,2); plot(times,Motor_Force_Array(:,2),col(4,:),LineWidth=1); grid on; hold on;
ylabel('F2'); xlim([0,max(times)]);
subplot(4,1,3); plot(times,Motor_Force_Array(:,3),col(5,:),LineWidth=1); grid on; hold on;
ylabel('F3'); xlim([0,max(times)]);
subplot(4,1,4); plot(times,Motor_Force_Array(:,4),col(6,:),LineWidth=1); grid on; hold on;
ylabel('F4'); xlim([0,max(times)]); xlabel('Time (s)');








%% Non-Linearized without feedback

% Run Simulation
[time,aircraft_state_array_] = ode45(@(t,var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces),t,var);

% Calculate Control Forces
control = [  -1,          -1,         -1,         -1      ; % Set Up Matrix 
          -d./sqrt(2), -d./sqrt(2), d./sqrt(2), d./sqrt(2); 
          d./sqrt(2), -d./sqrt(2), -d./sqrt(2), d./sqrt(2); 
              km,         -km,          km,        -km    ].* motor_forces;

% Map Control Force Values over Time Interval
control_input_array_(:,1) = repmat(sum(control(1,:)),length(time),1);
control_input_array_(:,2) = repmat(sum(control(2,:)),length(time),1);
control_input_array_(:,3) = repmat(sum(control(3,:)),length(time),1);
control_input_array_(:,4) = repmat(sum(control(4,:)),length(time),1);

fig = 1:6; % Set figure numbers
col = ['m--';'m--';'m--';'c--';'c--';'c--';'k--']; % Set colors for plotting


% Plot Non-linearized Simulation
PlotAircraftSim(time,aircraft_state_array_,control_input_array_,fig,col, '25')




Motor_Force_Array_(:,1) = repmat(motor_forces(1),length(time),1);
Motor_Force_Array_(:,2) = repmat(motor_forces(2),length(time),1);
Motor_Force_Array_(:,3) = repmat(motor_forces(3),length(time),1);
Motor_Force_Array_(:,4) = repmat(motor_forces(4),length(time),1);




figure (7); % Figure 7 for motor force
sgtitle('Motor Forces')
subplot(4,1,1); plot(time,Motor_Force_Array_(:,1),col(3,:),LineWidth=1); grid on; hold on;
ylabel('F1'); xlim([0,max(time)]);
subplot(4,1,2); plot(time,Motor_Force_Array_(:,2),col(4,:),LineWidth=1); grid on; hold on;
ylabel('F2'); xlim([0,max(time)]);
subplot(4,1,3); plot(time,Motor_Force_Array_(:,3),col(5,:),LineWidth=1); grid on; hold on;
ylabel('F3'); xlim([0,max(time)]);
subplot(4,1,4); plot(time,Motor_Force_Array_(:,4),col(6,:),LineWidth=1); grid on; hold on;
ylabel('F4'); xlim([0,max(time)]); xlabel('Time (s)');






exportgraphics(figure(7),'Motor_Forces_For_Roll_Rate_deviation_For_Both_Controlled_And_Uncontrolled.png')
exportgraphics(figure(6),'3D_Trajectory_For_Roll_Rate_deviation_For_Both_Controlled_And_Uncontrolled.png')
exportgraphics(figure(5),'Control_Inputs_For_Roll_Rate_deviation_For_Both_Controlled_And_Uncontrolled.png')
exportgraphics(figure(4),'Body_Angular_Rates_Components_For_Roll_Rate_deviation_For_Both_Controlled_And_Uncontrolled.png')
exportgraphics(figure(3),'Air_Relative_Velocity_Components_For_Roll_Rate_deviation_For_Both_Controlled_And_Uncontrolled.png')
exportgraphics(figure(2),'Body_Angular_Components_For_Roll_Rate_deviation_For_Both_Controlled_And_Uncontrolled.png')
exportgraphics(figure(1),'Inertial_Position_Components_For_Roll_Rate_deviation_For_Both_Controlled_And_Uncontrolled.png')
