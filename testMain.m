clear; clc; close all;

load('RSdata_nocontrol.mat')

time = rt_estim.time;
aircraft_state_array = rt_estim.signals.values;
control_input_array = rt_motor.signals.values;
fig = 1:6;
col = ['r','b','g','m','c','y'];



PlotAircraftSim(time,aircraft_state_array,control_input_array,fig, col)