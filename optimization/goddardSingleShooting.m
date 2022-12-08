function [cost] = goddardSingleShooting(controls, initialState, finalState, Rocket) 

    % Unpack Rocket Properties 
    thrust = Rocket.thrust; 
    effectiveExhaustVelocity = Rocket.effectiveExhaustVelocity; 
    throttleSmoothing = Rocket.throttleSmoothing; 

    % Define TimeSpan 
    initialEpoch = 0; 
    finalEpoch = abs(controls(4)); 
    epochs = [initialEpoch finalEpoch]; 
    
    % Build Adjoint State 
    initialCostates = controls(1:3); 
    adjointState = [initialState; initialCostates]; 
    
    % Set odeOptions 
    odeopts = odeset('RelTol', 1e-10, 'AbsTol', 1e-12, 'Events', @singularArcs); 
    timeHistory = initialEpoch;
    stateHistory = adjointState; 
    done = false; 
    while ~ done 

        % Compute initialValue of Switch Function and Derivative 
        [~, ~, S, Sdot] = rocketDynamics_bang(epochs(1),adjointState,thrust,effectiveExhaustVelocity,throttleSmoothing); 
        
        % Update timeSpan 
        timeSpan = epochs; 
        % Integrate Dynamics
        tolerance = 1e-3; 
        if S && Sdot < tolerance  
            [time, trajectory] = ode89(@rocketDynamics_singular, timeSpan, adjointState, odeopts, thrust, effectiveExhaustVelocity, throttleSmoothing);
        else
            [time, trajectory] = ode89(@rocketDynamics_bang, timeSpan, adjointState, odeopts, thrust, effectiveExhaustVelocity, throttleSmoothing);
        end

        % Check if final time is reached 
        done = (time(end) == finalEpoch); 
        
        % Set initialEpoch to finalEpoch of last leg 
        epochs = [time(end), finalEpoch]; 
        % Set adjointState to finalState of last leg 
        adjointState = trajectory(end,:)'; 
    
        % Append Trajectory Legs
        timeHistory = [timeHistory; time(2:end)]; 
        stateHistory = [stateHistory, trajectory(2:end,:)'];

    end 

    % Check Hamiltonian(tf) 
    [~, ~, ~, ~, Hamiltonian] = rocketDynamics_bang(timeHistory(end),stateHistory(:,end),thrust,effectiveExhaustVelocity,throttleSmoothing);

    % Build Cost 
    cost = stateHistory(1:3,end) - finalState; 
    cost = rmmissing(cost);
    cost(3) = trajectory(end,4); % Free final height : lam_h = 0
    cost(4) = Hamiltonian(end); % Free final Epoch : H(tf) = 0

end