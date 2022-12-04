%% Rocket Dynamics 
function [stateDerivative, height, velocity, acceleration, mass] = rocketDynamics(epochs, state, Rocket)

% Unpack inputs
[height, velocity, mass] = deal(state(1), state(2), state(3));

% Mass cannot fall below 0.6 since propellant is only 0.4*m0
if mass <= 0.6
    thrust = 0; 
else 
    thrust = Rocket.thrust;
end 
effectiveExhaustVelocity = Rocket.effectiveExhaustVelocity;

% Compute Drag  
commonTerm = 620; % rho0*Cd*A/(m0*g)
Beta = 500; 
drag = 0.5*velocity^2*commonTerm*exp(Beta*(1 - height)); 

% Compute Gravity Loss 
gravityLoss = 1/height^2; 

% % If Rocket is at h = 0, acceleration must be non-negative
% if height == 0 && thrust == 0 
%     acceleration = 0; 
% end 

% Compute Rocket acceleration 
acceleration = (thrust - drag)/mass - gravityLoss; 

% Compute Mass flow rate 
massFlowRate = -thrust/effectiveExhaustVelocity;

% Build State Derivative 
stateDerivative = [velocity; acceleration; massFlowRate]; 
end
