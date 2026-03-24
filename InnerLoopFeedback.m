function [Fc, Gc] = InnerLoopFeedback(var)
%{
Create a function to calculate the control vectors Fc and Gc. The function takes as input the 12x1
aircraft state var. The control force in the body 𝑧-direction should still equal the weight of the
quadrotor (hard code the values in the function along with the control gains). Set the control moment
using the control laws from Problem 3.1.
%}



g=9.81;
m=0.068;

L.k1 = ;
L.k2 = ;

M.k1 = ;
M.k2 = ;

L = -L.k1.*var(10)-L.k2.*var(4);
M =  -L.k1.*var(11)-L.k2.*var(5);
N= 0;
Fc=[0 0 -m*g];
Gc=[L M N];

end