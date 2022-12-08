close all; clear; clc; 

% Generate Symbolic Variables 
syms height velocity real  
syms thrust effectiveExhaustVelocity mass epoch rho real positive 
syms lam [3 1] real 

% State Vector 
x = [height, velocity, mass]'; 

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

% Switch Function Derivative 
Sdot = -lam1/mass + lam2*drag/mass^2*(2/velocity + 1/effectiveExhaustVelocity); 

% Optimal Thrust (Hyperbolic Tangential Smoothing) 
delta_opt = 0.5*(1 + tanh(S/rho));

% Compute Optimal Thrust Value 
T_opt = delta_opt*thrust;

%% Build State Derivative 
stateDerivative = [statePerturbation; costatePerturbation]; 

% Substitute Optimal Thrust Value 
stateDerivative = subs(stateDerivative, thrust, T_opt);

% Creat Symbolic File for Equations of Motion
matlabFunction(stateDerivative, T_opt, S, Sdot, Hamiltonian, 'file', 'rocketDynamics_bang', 'Optimize', true, 'vars', {epoch, [x;lam], thrust, effectiveExhaustVelocity, rho}); 
