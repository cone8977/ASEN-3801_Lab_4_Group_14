function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)
% for use by ode45 to simulate the full nonlinear equations of motion where: t is time; var is the 12 x
% 1 aircraft state vector; g is the acceleration due to gravity; m is mass; I is the inertia matrix; d, km,
% nu, and mu are the remaining quadrotor parameters; motor_forces = [f1; f2; f3; f4] is
% the 4 x 1 vector of motor forces, and var_dot is the 12 x 1 derivative of the state vector. Include
% attitude dynamics and kinematics using the Euler angle attitude representation. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  t: Time 
%  var: 12x1 state vector   [ x, y, z, phi, theta, psi, u, v, w, p, q, r] 
%  g: accel due to grav 
%  m: Mass
%  I: Inertia Matrix
%  d: 
%  km:
%  nu:
%  mu:
%  Motor Forces: [f1; f2; f3; f4] is the 4 x 1 vector of motor forces
%  var_dot: 12x1 derivative of the state vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
