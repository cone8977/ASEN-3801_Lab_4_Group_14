clear; clc; close all;

load('RSdata_nocontrol.mat') % load porvided data to test plotting

time = rt_estim.time;
aircraft_state_array = rt_estim.signals.values;
control_input_array = rt_motor.signals.values;
fig = 1:6; % set figure numbers
col = ['r','b','g','m','c','k']; % set colors for plotting, groups by translational and rotational motion/forces


% Test plotting
PlotAircraftSim(time,aircraft_state_array,control_input_array,fig, col)
