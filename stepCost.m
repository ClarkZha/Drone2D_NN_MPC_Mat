function cost = stepCost(command, currentState, goalState, Q, R)
stateDiff = goalState - currentState(1:4);
cost = command'* Q * command + stateDiff' * R * stateDiff;
end

