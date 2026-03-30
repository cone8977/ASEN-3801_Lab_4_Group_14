% Contributors: Cody Newton 
% Course number: ASEN 3801
% File name: QuadrotorEOM
% Created: 3/3/26
function var_dot=CL_Linearized_EOM(t,var,g,m,I)
%{
Inputs: 
    t: time vector
    var: 12x1 state vector
    g: gravity
    m: mass
    I: Inertia Matrix 
Outputs:
    var_dot: 12x1 state vector derivative

%}
%% Control Forces

[Fc, Gc] = InnerLoopFeedback(var);

%% Position Derivatives 
X_dot=var(7);
Y_dot=var(8);
Z_dot=var(9);

%% Angle derivatives 
Phi_dot=var(10);
Theta_dot=var(11);
Psi_dot=var(12);

%% Velocity derivatives 
vel_dot=g*([-var(5); var(4); 0]) + (1./m).*([0; 0; Fc(3)]);

%% angular velocity derivatives 
omega_dot=[1/I(1,1).*Gc(1); 1/I(2,2).*Gc(2); 1/I(3,3).*Gc(3)];

%% Output 
var_dot=[X_dot;Y_dot;Z_dot; Phi_dot;Theta_dot;Psi_dot; vel_dot;omega_dot];
end