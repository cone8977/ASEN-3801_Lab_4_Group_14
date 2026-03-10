function var_dot = QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc)
% for use by ode45 to simulate the linear equations of motion where: t is time; var is the 12 x
% 1 aircraft state vector; g is the acceleration due to gravity; m is mass; I is the inertia matrix;
% deltaFc is the in deviation in the forces from the steady hover trim condition (1x1)
% deltaGc is the in deviation in the moments from the steady hover trim condition (3x1)
% Include attitude dynamics and kinematics using the Euler angle attitude representation. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  t: Time 
%  var: 12x1 state vector   [ x, y, z, phi, theta, psi, u, v, w, p, q, r] 
%  g: accel due to grav 
%  m: Mass
%  I: Inertia Matrix
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

%% Inertia Struct
In.x=I(1,1);
In.y=I(2,2);
In.z=I(3,3);

%% Disturbances
Disturb.Z = deltaFc;
Disturb.L = deltaGc(1);
Disturb.M = deltaGc(2);
Disturb.N = deltaGc(3);

%% X_dot, Y_dot, Z_dot
X_dot = State.u;
Y_dot = State.v;
Z_dot = State.w;


%% Phi_dot, Theta_dot, Psi_dot
Phi_dot = State.p;
Theta_dot = State.q;
Psi_dot = State.r;


%% U_dot, V_dot, W_dot
U_dot = -State.theta * g;
V_dot = State.phi * g;
W_dot = Disturb.Z / m;


%% P_dot, Q_dot, R_dot
P_dot = Disturb.L/In.x;
Q_dot = Disturb.M/In.y;
R_dot = Disturb.N/In.z;

%% Compilation
var_dot=[ X_dot; Y_dot; Z_dot; Phi_dot; Theta_dot; Psi_dot; U_dot; V_dot; W_dot; P_dot; Q_dot; R_dot];

end