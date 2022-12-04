function [stateDerivative,thrust,S] = rocketDynamics_symbolic(epoch,in2,thrust,effectiveExhaustVelocity)
%rocketDynamics_symbolic
%    [stateDerivative,THRUST,S] = rocketDynamics_symbolic(EPOCH,IN2,THRUST,effectiveExhaustVelocity)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    03-Dec-2022 22:13:19

height = in2(1,:);
lam1 = in2(4,:);
lam2 = in2(5,:);
lam3 = in2(6,:);
mass = in2(3,:);
velocity = in2(2,:);

% Check if fuel is spent 
if mass <= 0.6 
    thrust = 0; 
end

t2 = velocity.^2;
t3 = 1.0./effectiveExhaustVelocity;
t4 = 1.0./mass;
t5 = height.*5.0e+2;
t6 = -t5;
t7 = t6+5.0e+2;
t8 = exp(t7);
t9 = t2.*t8.*3.1e+2;
stateDerivative = [velocity;-t4.*(t9-thrust)-1.0./height.^2;-t3.*thrust;-lam2.*(1.0./height.^3.*2.0+t2.*t4.*t8.*1.55e+5)+1.0;-lam1+lam2.*t4.*t8.*velocity.*6.2e+2;-lam2.*t4.^2.*(t9-thrust)];
if nargout > 1
    thrust = thrust;
end
if nargout > 2
    S = lam2.*t4-lam3.*t3;
end
