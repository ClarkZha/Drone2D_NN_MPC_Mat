function [command, state] = droneMPC(dt, horizon, initState, goalState, Q, R, F, maxAcc, maxAngAcc)
grav = 9.81;

%% Setup the optimization problem 
optState = sdpvar(6,horizon); %
optCommand = sdpvar(2,horizon-1);


% optStateGuess = repmat(initState,1,horizon);
optCommandGuess = repmat([grav;0],1,horizon-1);
% assign(optState,optStateGuess); 
assign(optCommand,optCommandGuess);



cost = 0;
constraint = [optState(:,1) == initState];

for i=1:horizon-1
    cost = cost + stepCost(optCommand(:,i), optState(:,i), goalState, Q, R);
    nextState = stepDynamics(dt, optState(:,i), optCommand(:,i), false);
    constraint = [constraint; nextState-optState(:,i+1) == [0;0;0;0;0;0]];
end

% Final state cost
cost = cost + (optState(1:4,horizon)- goalState)' * F * (optState(1:4,horizon)- goalState);

% Constraint the command input
constraint = [constraint; optCommand(1,:) <= ones(1,horizon-1)*maxAcc; optCommand(1,:) >= zeros(1,horizon-1)];
constraint = [constraint; optCommand(2,:) <= ones(1,horizon-1)*maxAngAcc; optCommand(2,:) >= -ones(1,horizon-1)*maxAngAcc];


%% Solve the optimization problem
options = sdpsettings('solver','fmincon','verbose', 0, 'fmincon.maxfunevals', 1000, 'fmincon.maxiter', 1000);
sol = optimize(constraint, cost, options);
command = double(optCommand);
state = double(optState);
end
