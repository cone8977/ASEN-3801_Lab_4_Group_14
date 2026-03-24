clc;
clear; 
close all;

%%%%%%
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





