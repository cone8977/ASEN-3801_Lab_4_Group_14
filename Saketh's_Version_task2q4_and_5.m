clc;
clear; 
close all;
% task 2 question 4



function motor_forces = ComputeMotorForces (Fc, Gc, d, km)

%Fc is the force vector [Xc Yc Zc]'

% using the last equation in slide 9





D = [[Fc(3) Gc(1) Gc(2) Gc(3)]]' 
%This is from the Control variables from the left hand side of slide 9,
%where Zc, Lc, Mc, Nc corresponds to the values in D respectively

A_cl = [-1 -1 -1 -1;
    -d/(2^(1/2)) -d/(2^(1/2)) d/(2^(1/2)) d/(2^(1/2));
    d/(2^(1/2)) -d/(2^(1/2)) -d/(2^(1/2)) d/(2^(1/2));
    km -km km -km]

%This is the 4x4 matrix in the equation

motor_forces = inv(A_cl) * D
end


% Task 2 Question 5

% note added km and d as a function parameter here even though the lab
% document did not have it in the parameters becuase it is needed for the
% other functions inside this function

function var_dot = QuadrotorEOMwithRateFeedback (t, var, g, m, I, nu, mu, km, d)

% This is needed to compute the control forces and moments (this is the
% function from 2.3)

[Fc, Gc] = RotationalDerivativeFeedback (var, m, g)

%Needed to compute motor forces (this is the function from 2.4)
motor_forces = ComputeMotorForces (Fc, Gc, d, km)

%outputs the var_dot (this is the function from 1.2)
var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)
end






%{ 
NOTE THIS IS SOMETHING I CONTINUED BUT SAVING FOR LATER JUST INCASE WE
%MIGHT NEED IT, I DIDNT REALLY UNDERSTAND WHAT THE QUESTION WAS SAYING SO I
%THINK I CONTINUED TO REDO 1.2 IN MY OWN WAY WITHOUT REALIZING PROPERLY







% Task 2 Question 5

function var_dot = QuadrotorEOMwithRateFeedback (t, var, g, m, I, nu, mu)

% This is needed to compute the control forces and moments
[Fc, Gc] = RotationalDerivativeFeedback (var, m, g)

motor_forces = ComputeMotorForces (Fc, Gc, d, km)

%Control Forces and moments
Zc = Fc(3)


%Note all these equations are available in slide 9

% components of the position from the state vector var
xe = var (1);
ye = var (2);
ze = var (3);

% components of the roll pitch yaw from the state vector var
phi = var (4);
theta = var (5);
psi = var (6);

% components of the air relative velocity vector from the state vector var
u = var (7);
v = var (8);
w = var (9);


% components of the angular velocity from the state vector var
p = var (10);
q = var (11);
r = var (12);

% Moment of inertia components from the moment of inertia vector
Ix = I(1);
Iy = I(2);
Iz = I(3);

%Va magnitude
Va_m = ( u^(2) + v^(2) + w^(2) )^ (1/2)


%uncontrollable forces
[X Y Z] = -1* nu * Va_m .* [u v w]

%uncontrollable moments
[L M N] = -1* mu * ( ( p^(2) + q^(2) + r^(2) )^ (1/2) )* [p q r]

% derivative of position
xe_dot = u * ( cos(theta)*cos(psi) +  sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi) )
ye_dot = v * ( cos(theta)*sin(psi) +  sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi) )
ze_dot = w * (-sin(theta) + sin(phi)*cos(theta) + cos(phi)*cos(theta) )

% derivative of euler angles
phi_dot = p * (1 + sin(phi)*tan(theta) + cos(phi)*tan(theta) )
theta_dot = q * (cos(phi) - sin(phi) )
psi_dot = r * (sin(phi)*sec(theta) + cos(phi)*sec(theta) )

% derivative of velocity

u_dot = r*v - q*u -g*sin(theta) + X/m
v_dot = p*w - r*u +g*sin(phi)*cos(theta) + Y/m
w_dot





end

%}



