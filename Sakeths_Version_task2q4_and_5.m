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

function var_dot = QuadrotorEOMwithRateFeedback (t, var, g, m, I, nu, mu)


% From function QuadrotorEOM but Fc and Gc from 2.3 instead of having km
% and D and computing the motor forces and all


[Fc, Gc] = RotationDerivativeFeedback(var, m, g)

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
%% Trig structure 
% Psi
Trig.cpsi=cos(State.psi);
Trig.spsi=sin(State.psi);
Trig.tpsi=tan(State.psi);
% Theta
Trig.ctheta=cos(State.theta);
Trig.stheta=sin(State.theta);
Trig.ttheta=tan(State.theta);
% Phi
Trig.cphi=cos(State.phi);
Trig.sphi=sin(State.phi);
Trig.tphi=tan(State.phi);

%% Inetria Struct
In.x=I(1,1);
In.y=I(2,2);
In.z=I(3,3);


%This is the change done relative to the quadrotorEOM, the Zc, Lc, Mc, Nc
%are computed using Fc and Gc rather than using motor forces and all (as
%you can see below this chunk is the commented section of the original
%quadrotorEOM.

Control.Z = Fc(3)
Control.L = Gc(1)
Control.M = Gc(2)
Control.N = Gc(3)



% %% Control Struct
% control=[-1 -1 -1 -1; -d./sqrt(2) -d./sqrt(2) d./sqrt(2) d./sqrt(2); d./sqrt(2) -d./sqrt(2) -d./sqrt(2) d./sqrt(2); km -km km -km].*motor_forces;
% Control.Z=sum(control(1,:));
% Control.L=sum(control(2,:));
% Control.M=sum(control(3,:));
% Control.N=sum(control(4,:));



%% Aero Struct
Force= -nu.*norm([State.u State.v State.w]).*[State.u; State.v; State.w];
Moments=-mu.*norm([State.p State.q State.r]).*[State.p; State.q; State.r];
Aero.X=Force(1);
Aero.Y=Force(2);
Aero.Z=Force(3);
Aero.L=Moments(1);
Aero.M=Moments(2);
Aero.N=Moments(3);

%% X_dot, Y_dot, Z_dot
Pos_dot=[Trig.ctheta.*Trig.cpsi, (Trig.sphi.*Trig.stheta.*Trig.cpsi)-(Trig.cphi.*Trig.spsi), (Trig.cphi.*Trig.stheta.*Trig.cpsi)-(Trig.sphi.*Trig.spsi); ...
                     Trig.ctheta.*Trig.spsi, (Trig.sphi.*Trig.stheta.*Trig.spsi)+(Trig.cphi.*Trig.cpsi), (Trig.cphi.*Trig.stheta.*Trig.spsi)-(Trig.sphi.*Trig.cpsi); ...
                     -Trig.stheta, Trig.ctheta.*Trig.sphi, Trig.ctheta.*Trig.cphi] * [State.u;State.v;State.w];

X_dot = Pos_dot(1);
Y_dot = Pos_dot(2);
Z_dot = Pos_dot(3);


%% Phi_dot, Theta_dot, Psi_dot
Angle_dot=[ 1, Trig.sphi.*Trig.ttheta, Trig.cphi.*Trig.ttheta;
            0, Trig.cphi, -Trig.sphi
            0, Trig.sphi./Trig.ctheta, Trig.cphi./Trig.ctheta]*[State.p;State.q;State.r;]; 


Phi_dot = Angle_dot(1);
Theta_dot = Angle_dot(2);
Psi_dot = Angle_dot(3);


%% U_dot, V_dot, W_dot

v_dot= cross([State.u,State.v,State.w],[State.p,State.q,State.r])' + g.*[-Trig.stheta;Trig.ctheta.*Trig.sphi ;Trig.ctheta.*Trig.cphi] +[Aero.X; Aero.Y; Aero.Z]./m+[0;0;Control.Z]./m;

U_dot = v_dot(1);
V_dot = v_dot(2);
W_dot = v_dot(3);


%% P_dot, Q_dot, R_dot
Omega_dot= [ ((In.y-In.z)./In.x).*State.q.*State.r;((In.z-In.x)./In.y).*State.p.*State.r;((In.x-In.y)./In.z).*State.q.*State.p]+[(1/In.x).*Aero.L; (1/In.y).*Aero.M; (1/In.z).*Aero.N]+[(1/In.x).*Control.L; (1/In.y).*Control.M; (1/In.z).*Control.N];

P_dot = Omega_dot(1);
Q_dot = Omega_dot(2);
R_dot = Omega_dot(3);

% %% Ground Collison
% if(State.z>=0)        
%     Z_dot=0; 
%     U_dot=0;
%     V_dot=0;
%     W_dot=0;
%     P_dot=0;
%     Q_dot=0;
%     R_dot=0;
% end

%% Compilation
var_dot=[ X_dot; Y_dot; Z_dot; Phi_dot; Theta_dot; Psi_dot; U_dot; V_dot; W_dot; P_dot; Q_dot; R_dot];


end








% Code trial 

% % Task 2 Question 5
% 
% % note added km and d as a function parameter here even though the lab
% % document did not have it in the parameters becuase it is needed for the
% % other functions inside this function
% 
% function var_dot = QuadrotorEOMwithRateFeedback (t, var, g, m, I, nu, mu, km, d)
% 
% % This is needed to compute the control forces and moments (this is the
% % function from 2.3)
% 
% [Fc, Gc] = RotationalDerivativeFeedback (var, m, g)
% 
% %Needed to compute motor forces (this is the function from 2.4)
% motor_forces = ComputeMotorForces (Fc, Gc, d, km)
% 
% %outputs the var_dot (this is the function from 1.2)
% var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)
% 
% end
% 





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



