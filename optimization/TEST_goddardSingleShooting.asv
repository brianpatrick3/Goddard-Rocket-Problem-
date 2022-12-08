function [cost] = TEST_goddardSingleShooting(controls, initialState, finalState, Rocket) 

    % Unpack Rocket Properties 
    thrust = Rocket.thrust; 
    effectiveExhaustVelocity = Rocket.effectiveExhaustVelocity; 

    % Define TimeSpan 
    initialEpoch = 0; 
    finalEpoch = abs(controls(4)); 
    epochs = [initialEpoch finalEpoch]; 
    
    % Build Adjoint State 
    initialCostates = controls(1:3); 
    adjointState = [initialState; initialCostates]; 
    
    % Set odeOptions 
    odeopts = odeset('RelTol', 1e-10, 'AbsTol', 1e-12);  

    % Integrate Dynamics
    [time, trajectory] = ode89(@rocketDynamics_TEST, epochs, adjointState, odeopts, thrust, effectiveExhaustVelocity);

    % Propagate trajectory 
    thrustHistory = zeros(1,length(time)); 
    switchHistory = zeros(1,length(time)); 
    SdotHistory = zeros(1,length(time)); 
    HamiltonianHistory = zeros(1,length(time));
    HamiltonianHistory = zeros(1,length(time)); 
    stateDerivativeHistory = zeros(6, length(time)); 
    for i = 1:length(time) 
        [stateDerivativeHistory(i,:), thrustHistory(i), switchHistory(i), SdotHistory(i), HamiltonianHistory(i)] = rocketDynamics_TEST(time(i),trajectory(i,:)',thrust,effectiveExhaustVelocity);
    end 

    % Build Cost 
    cost = trajectory(end, 1:3)' - finalState; 
    cost = rmmissing(cost);
    cost(3) = trajectory(end,4); % Free final height : lam_h = 0
    cost(4) = HamiltonianHistory(end); % Free final Epoch : H(tf) = 0

end