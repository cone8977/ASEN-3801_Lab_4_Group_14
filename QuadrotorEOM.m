% Contributors: Cody Newton, Víctor Turpin Aguayo, Liz Thompson
% Course number: ASEN 3801
% File name: QuadrotorEOM
% Created: 3/3/26

function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs:   t = Simulation time (s)
%           var = 12x1 QR state vector [x,y,z,phi,theta,psi,u,v,w,p,q,r] 
%           g = Acceleration due to gravity (m/s^2)
%           m = QR Mass (kg)
%           I = QR Inertia Matrix (kg*m^2)
%           d = Radial distacne from CG to propeller (m)
%           km = Control moment coefficient (N*m/(N))
%           nu = Aerodynamic force coefficient (N/(m/s)^2)
%           mu = Aerodynamics moment coefficient (N*m/(rad/s)^2)
%           Motor Forces = 4 x 1 motor force vector [f1; f2; f3; f4]
% 
% Outputs:  var_dot: 12x1 derivative of the state vector
%
% Methodology: For use by ode45 to simulate nonlinear QR EOMs. Compute
% derivative of each aircraft state using the QR EOMs.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
%%
Trig.cpsi=cos(State.psi);
Trig.spsi=sin(State.psi);
Trig.ctheta=cos(State.theta);
Trig.stheta=sin(State.theta);
Trig.cphi=cos(State.phi);
Trig.sphi=sin(State.phi);

%% X_dot, Y_dot, Z_dot
Pos_dot=[Trig.ctheta.*Trig.cpsi, (Trig.sphi.*Trig.stheta.*Trig.cpsi)-(Trig.cphi.*Trig.spsi), (Trig.cphi.*Trig.stheta.*Trig.cpsi)-(Trig.sphi.*Trig.spsi); ...
                     Trig.ctheta.*Trig.spsi, (Trig.sphi.*Trig.stheta.*Trig.spsi)+(Trig.cphi.*Trig.spsi), (Trig.cphi.*Trig.stheta.*Trig.spsi)-(Trig.sphi.*Trig.cpsi); ...
                     -Trig.stheta, Trig.ctheta.*Trig.sphi, Trig.ctheta.*Trig.cphi] * [State.u,State.v,State.w];

X_dot = Pos_dot(1);
Y_dot = Pos_dot(2);
Z_dot = Pos_dot(3);










%% Compilation
var_dot=[ X_dot; Y_dot; Z_dot; Phi_dot; Theta_dot; Psi_dot; U_dot; V_dot; W_dot; P_dot; Q_dot; R_dot];


end
