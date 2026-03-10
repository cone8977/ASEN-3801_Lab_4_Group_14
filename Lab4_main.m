clc 
clear 
close all 
%% Constants/Varibales 
t= [0 10];
var=[10; 15 ;-20;
     0 ;0 ; pi/2; 
     0 ;0 ;0;
     0 ;0 ;0];
g=9.81;
m=0.068;
I=[5.8*10^(-5),0,0;
   0,7.2*10^(-5),0;
   0,0,1.0*10^(-4)];
d=0.06; 
km=0.0024;
nu=1*10^(-3);
mu=2*10^(-6);
motor_forces=[m*g/4 m*g/4 m*g/4 m*g/4]';


%% function 

func=@(t,var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces);
[T,y] = ode45(func,t,var);
[~, Control] = cellfun(func, num2cell(T), num2cell(y,2),'UniformOutput',0);
Z_c = zeros(length(T),1); 
L_c = zeros(length(T),1);
M_c = zeros(length(T),1);
N_c = zeros(length(T),1);

for i = 1:length(T)
    Z_c(i) = Control{i}.Z;
    L_c(i) = Control{i}.L;
    M_c(i) = Control{i}.M;
    N_c(i) = Control{i}.N;
end

PlotAircraftSim(T,y,[Z_c,L_c,M_c,N_c],[1 2 3 4 5 6],['b' 'b' 'b' 'b' 'b' 'b'])