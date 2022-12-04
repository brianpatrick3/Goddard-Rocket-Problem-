close all; clear; clc; 

% Generate Symbolic Variables 
syms height velocity real  
syms thrust effectiveExhaustVelocity mass epoch real positive 
syms lam [3 1] real 

% State Vector 
x = [height, velocity, mass]'; 

% % Propellant mass = 0.4 so total mass cant go below 0.6 
% if mass <= 0.6 
%     thrust = 0; 
% end

% thrust = 0.5*sign(1 + (mass - 0.6));

% thrust = evalin(symengine, 'piecewise([mass > 0.6, thrust], [mass <= 0.6, 0])');

% Compute Drag 
commonTerm = 620; 
Beta = 500; 
drag = 0.5*velocity^2*commonTerm*exp(Beta*(1 - height)); 

% Compute Gravity Loss 
gravityLoss = 1/height^2; 

% Compute Rocket Acceleration 
acceleration = (thrust - drag)/mass - gravityLoss; 

% Comute Mass Flow Rate 
massFlowRate = -thrust/effectiveExhaustVelocity; 

% Define Rocket Dynamics 
xdot = [velocity; acceleration; massFlowRate]; 

% State Dynamics 
statePerturbation = xdot; 

% Cost 
cost = -height;

% Formulate Hamiltonian 
Hamiltonian = cost + lam'*xdot; 

% Costate Equations 
costatePerturbation = transpose(-jacobian(Hamiltonian, x)); 

% Switch Function 
S = (lam2/mass - lam3/effectiveExhaustVelocity);

% Optimal Thrust 
T_opt = 0.5*(1 + sign(S)); 

% Build State Derivative 
stateDerivative = [statePerturbation; costatePerturbation]; 

% Creat Symbolic File for Equations of Motion
matlabFunction(stateDerivative, thrust, S, 'file', 'rocketDynamics_symbolic', 'Optimize', true, 'vars', {epoch, [x;lam], thrust, effectiveExhaustVelocity}); 
