function cost = computeCostRollOut(dt, horizon, initState, goalState, command, costParam, quadParam)
% Rollout the dynamics and compute cost. 
simFlag = false;
Q = costParam.Q;
R = costParam.R;
F = costParam.F;
cost = 0;
currentState = initState;

for i=1:horizon-1
    cost = cost + stepCost(command(:,i), currentState, goalState, Q, R);
    currentState = stepDynamics(dt, currentState, command(:,i), quadParam, simFlag);
end

% Final state cost
cost = cost + (currentState(1:4)- goalState)' * F * (currentState(1:4)- goalState);
end
