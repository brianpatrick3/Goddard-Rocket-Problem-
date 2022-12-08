function [stateDerivative, thrust, S, Sdot, Hamiltonian] = rocketDynamics_TEST(epochs, state, Tmax, effectiveExhaustVelocity)
% test values
% state = [1.05 0.2 0.8 0 2 0]'; 
% Tmax = 3.5; 
% effectiveExhaustVelocity = 0.5; 

% Generate Symbolic thrust Variables  
syms thrust real positive  

% Unpack inputs
[height, velocity, mass] = deal(state(1), state(2), state(3)); 
[lam1, lam2, lam3] = deal(state(4), state(5), state(6)); 

% State Vector 
x = [height, velocity, mass]';
lam = [lam1, lam2, lam3]';

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
P1dot = 1 - lam2*(2/height^3 + (155000*velocity^2*exp(500 - 500*height))/mass); 
P2dot = (620*lam2*velocity*exp(500 - 500*height))/mass - lam1;
P3dot = (lam2*(- 310*exp(500 - 500*height)*velocity^2 + thrust))/mass^2; 

% Build Costate Perturbation 
costatePerturbation = [P1dot, P2dot, P3dot]'; 

% Switch Function 
S = (lam2/mass - lam3/effectiveExhaustVelocity);

% Switch Function Derivative 
Sdot = -lam1/mass + lam2*drag/mass^2*(2/velocity + 1/effectiveExhaustVelocity);

%% Singular Control 
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

% Compute Thrust value 
if S && Sdot < 1e-5
    thrust = Tsingular; 
else 
    thrust = Tmax*sign(S); 
end 

thrust = Tmax; 

if mass <= 0.6
    thrust = 0; 
end 

% Evaluate the stateDerivative at proper thrust value
stateDerivative = eval(stateDerivative); 
Hamiltonian = eval(Hamiltonian); 


end 
