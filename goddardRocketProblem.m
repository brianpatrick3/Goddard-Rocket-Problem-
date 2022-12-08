%%% File Name: goddardRocketProblem.m 
% Written By: Brian Patrick 
% Date Written: 11.21.2022 
% Description: This file is used to solve the Goddard Rocket Problem. 
%              The goal of this work is to determine the control input
%              needed to maximize the final altitude of a rocket in 1-D
%              motion under atmospheric drag and point mass gravitational
%              field.
close; clear all; clc; 
tic 

% Units 
LU = CelestialBodyConstants.EARTH_RADIUS; 
mu = GravitationalParameter.EARTH; 

% Rocket Properties
thrust = 3.5; 
effectiveExhaustVelocity = 0.5; 
% Rocket.dynamicPressureMax = 10; 

% Final Mass 60% of unity
finalMass = 0.6; 

% Set initial State [height, velocity, mass] 
initialState = [1 0 1]'; % Height starts at 1 since it begins at the Earth's surface 

% Initialize Costates 
initialCostate = [0 2 0]';
% initialCostate = rand(3,1); 

% Build Adjoint State 
adjointState = [initialState; initialCostate]; 

% Set timespan
epochs = [0 0.3]; % This is a guess, but the final time is free 

% Smoothing Parameter
throttleSmoothing = 1; 

% Integrator options
odeopts = odeset('RelTol', 1e-10, 'AbsTol', 1e-12, 'Events', @singularArcs, 'Events', @events);
% Propagate dynamics
[time, trajectory] = ode45(@rocketDynamics_bang, epochs, adjointState, odeopts, thrust, effectiveExhaustVelocity, throttleSmoothing); 

% Propagate Trajectory 
[stateDerivative,thrustHistory, switchFunction, swtichFunctionDerivative, Hamiltonian] = rocketDynamics_bang(time,trajectory',thrust,effectiveExhaustVelocity,throttleSmoothing);

%% Results 
% Maximum Rocket Altitude 
maxAltitude = (max(trajectory(:,1))*LU) - LU; % Km
fprintf('\n Maximum Rocket Altitude = %skm \n', num2str(maxAltitude))

%% Plotting 

% Plot Altitude
figure()
plot(time,trajectory(:,1), 'LineWidth', 1.5)
xlabel('Time') 
ylabel('Altitude')
set(findall(gcf, '-property', 'FontSize'), 'FontSize', 16)

% Plot Thrust 
figure()
plot(time,thrustHistory, 'LineWidth', 1.5)
xlabel('Time') 
ylabel('Thrust')
set(findall(gcf, '-property', 'FontSize'), 'FontSize', 16)

% Plot Mass 
figure() 
plot(time, trajectory(:,3), 'LineWidth', 1.5)
xlabel('Time') 
ylabel('Mass') 
set(findall(gcf,'-property', 'FontSize'), 'FontSize', 16)

% Plot Switch Function 
figure() 
plot(time, switchFunction, 'LineWidth', 1.5) 
xlabel('Time') 
ylabel('Switch Function') 
set(findall(gcf,'-property', 'FontSize'), 'FontSize', 16)

% Plot Hamiltonian
figure() 
plot(time, Hamiltonian, 'LineWidth', 1.5) 
xlabel('Time') 
ylabel('Hamiltonian') 
set(findall(gcf,'-property', 'FontSize'), 'FontSize', 16)

% Plot Costates
% Height Costate 
figure()
subplot 311 
plot(time, trajectory(:,4), 'LineWidth', 1.5) 
xlabel('Time')  
ylabel('\lambda_{h}')
% Velocity Costate
subplot 312 
plot(time, trajectory(:,5), 'LineWidth', 1.5) 
xlabel('Time') 
ylabel('\lambda_{v}') 
% Mass Costate 
subplot 313 
plot(time, trajectory(:,6), 'LineWidth', 1.5) 
xlabel('Time') 
ylabel('\lambda_{m}')

% Set Font Size 
set(findall(gcf,'-property', 'FontSize'), 'FontSize', 16)


%% Record Runtime 
runtime = toc; 
fprintf('\n Program Runtime: %ss \n', num2str(runtime))

%% Events 
function [value,isterminal,direction] = events(t,x, varargin)
% Locate the time when height passes through zero in a decreasing direction
% and stop integration.
value = x(3)-0.6;     % detect height = 0
isterminal = 1;   % stop the integration
direction = -1;   % negative direction
end 

