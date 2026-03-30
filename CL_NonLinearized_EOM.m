% Contributors: Cody Newton 
% Course number: ASEN 3801
% File name: QuadrotorEOM
% Created: 3/3/26
function var_dot = CL_NonLinearized_EOM(t,var,g,m,I,d,km,nu,mu) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs:   time = simulation time
%           var = 12x1 state vector [x,y,z,phi,theta,psi,u,v,w,p,q,r] 
%           g = acceleration due to gravity (m/s^2)
%           m = QR mass (kg)
%           I= QR inertia matrix (km*m^2)
%           d = Radial distance from cg to propeller (m)
%           km = Control moment coefficient (N*m/(N))
%           nu = Aerodynamic force coefficient (N/(m/s)^2)
%           mu = Aerodynamic moment coefficient (N*m/(rad/s)^2)
%          
% 
% Output:   var_dot: 12x1 derivative of the state vector
%

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

[Fc, Gc] = InnerLoopFeedback(var);
% Allocate Control Force/Moment Values
Control.Z=sum(Fc(3)); Control.L=sum(Gc(1));
Control.M=sum(Gc(2)); Control.N=sum(Gc(3));

%% Calcaulte Aerodynamic Force/Moments

% Calculate Aerodynamic Forces/Moments
Force = -nu.*norm([State.u State.v State.w]).*[State.u; State.v; State.w];
Moments = -mu.*norm([State.p State.q State.r]).*[State.p; State.q; State.r];

% Allocate Aerodynamic Force/Moment Values
Aero.X=Force(1); Aero.Y=Force(2); Aero.Z=Force(3);
Aero.L=Moments(1); Aero.M=Moments(2); Aero.N=Moments(3);

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