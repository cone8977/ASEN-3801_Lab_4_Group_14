function [Fc, Gc] = RotationDerivativeFeedback(var, m, g)
%{
Create a function to calculate the control vectors Fc and Gc. The function takes as input the 12x1
aircraft state var, aircraft mass m, and gravitational acceleration g. The control force in the body 𝑧𝑧-
direction should still equal the weight of the quadrotor. Set the control moments about each body
axis proportional to the rotational rates about their respective axes, but in the opposite sign of the
angular velocity with a gain of 0.004 Nm/(rad/sec):

Inputs:
Var: 12x1 state vector 
m: mass
g: Gravity 

Outputs:
FC: Control Forces
GC: Control Moments 

%}

GC_gain=0.004; %[Nm]

%% Moments
Gc = var(10:12).*-GC_gain;

%% Forces
Z = m.*g;

Fc=[0;0;Z];
end 

