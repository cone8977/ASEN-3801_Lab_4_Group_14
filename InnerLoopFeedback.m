% Contributors: Cody Newton 
% Course number: ASEN 3801
% File name: QuadrotorEOM
% Created: 3/3/26

function [Fc, Gc] = InnerLoopFeedback(var)
%{
Create a function to calculate the control vectors Fc and Gc. The function takes as input the 12x1
aircraft state var. The control force in the body 𝑧-direction should still equal the weight of the
quadrotor (hard code the values in the function along with the control gains). Set the control moment
using the control laws from Problem 3.1.
Inputs:
    var: 12x1 state vector 
Outputs: 
    Fc: Control Forces 3x1 vector (X,Y,Z)
    Gc: Control Moments 3x1 Vector (L,M,N)
%}

I=[5.8*10^(-5),0,0;
   0,7.2*10^(-5),0;
   0,0,1.0*10^(-4)];

g=9.81;
m=0.068;

L.k1 = 2.5.*I(1,1);
L.k2 = 1.*I(1,1);

M.k1 = 2.5.*I(2,2);
M.k2 = 1.*I(2,2);

Lc = -L.k1.*var(10)-L.k2.*var(4);
Mc =  -M.k1.*var(11)-M.k2.*var(5);
Nc= 0;
Fc=[0 0 -m*g];
Gc=[Lc Mc Nc];

end