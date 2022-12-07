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

% Optimal Thrust (Hyperbolic Tangential Smoothing) 
delta_opt = 0.5*(1 + tanh(S/rho));

% Compute Optimal Thrust Value 
T_opt = delta_opt*thrust;

%% Singular Arc

% Compute common terms 
[P1dot, P2dot, P3dot] = deal(costatePerturbation(1), costatePerturbation(2), costatePerturbation(3)); 
K = (2/velocity + 1/effectiveExhaustVelocity); 
Sdot_P1 = -1/mass; 
Sdot_P2 = drag/mass^2 * (K); 
Sdot_D = lam2/mass^2*(K); 
Sdot_m = lam1/mass^2 - 2*lam2*drag/mass^3 * (K); 
Sdot_v = 2*lam2*drag/velocity/mass^2*(1/velocity + 1/effectiveExhaustVelocity); 

% Compute Grouped Terms 
A = Sdot_D*(2*drag^2/velocity/mass + 2*drag/(height^2*velocity) + velocity*Beta*drag) + Sdot_v*(drag/mass + 1/height^2) - Sdot_P1*P1dot - Sdot_P2*P2dot;
B = (Sdot_D*2*drag/mass/velocity - Sdot_m/effectiveExhaustVelocity + Sdot_v/mass); 

% Compute Tsingular
Tsingular = A/B; 

%% Build State Derivative 
stateDerivative = [statePerturbation; costatePerturbation]; 

% Substitute Optimal Thrust Value 
stateDerivative = subs(stateDerivative, thrust, T_opt);

% Creat Symbolic File for Equations of Motion
matlabFunction(stateDerivative, thrust, S, Hamiltonian, Tsingular, 'file', 'rocketDynamics_symbolic', 'Optimize', true, 'vars', {epoch, [x;lam], thrust, effectiveExhaustVelocity, rho}); 
