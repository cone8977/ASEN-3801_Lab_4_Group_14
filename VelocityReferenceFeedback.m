% Contributors: Cody Newton 
% Course number: ASEN 3801
% File name: QuadrotorEOM
% Created: 3/3/26
function [Fc, Gc] = VelocityReferenceFeedback(t, var)
%{
Inputs: 
    t: time vector
    var: 12x1 state vector 
Outputs: 
    Fc: Control Force vector (3x1 (X,Y,Z))
    Gc: Control Moment vector (3x1 (L,M,N))
%}

I=[5.8*10^(-5),0,0;
   0,7.2*10^(-5),0;
   0,0,1.0*10^(-4)];

g=9.81;
m=0.068;
V_ref=;
U_ref=;


L.k1 = 2.5.*I(1,1);
L.k2 = 1.*I(1,1);
L.k3=;

M.k1 = 2.5.*I(2,2);
M.k2 = 1.*I(2,2);
M.k3=;

Lc = -L.k1.*var(10)-L.k2.*var(4)+L.k3.*(V_ref-var(8));
Mc =  -M.k1.*var(11)-M.k2.*var(5)+M.k3.*(V_ref-var(7));
Nc= 0;

Fc=[0 0 -m*g];
Gc=[Lc Mc Nc];

end