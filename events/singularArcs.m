function [value,isterminal,direction] = singularArcs(t,x, varargin)
% Locate the time when height passes through zero in a decreasing direction
% and stop integration.

% Unpack inputs 
adjointState = x; 
[thrust, effectiveExhaustVelocity, throttleSmoothing] = deal(cell2mat(varargin(1)), cell2mat(varargin(2)), cell2mat(varargin(3))); 

% Compute Switch Function and Switch Function Derivative Value 
[~,~,switchFunction,switchFunctionDerivative,~] = rocketDynamics_bang(t,adjointState,thrust,effectiveExhaustVelocity,throttleSmoothing);

value = switchFunction + switchFunctionDerivative;     % detect S + Sdot = 0
isterminal = 1;   % stop the integration
direction = 0;   % Either direction
end 