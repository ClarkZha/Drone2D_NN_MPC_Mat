function [command, state] = droneMPC(dt, horizon, initState, goalState, costParam, quadParam, optCommandGuess)
% Solve MPC for the planar drone problem


simFlag = false;
Q = costParam.Q;
R = costParam.R;
F = costParam.F;

grav = quadParam.grav;
maxThrust = quadParam.maxThrust;

%% Setup the optimization problem 
optState = sdpvar(6,horizon); %
optCommand = sdpvar(2,horizon-1); %

% The optCommandGuess is an optional parameter. Initialize it if it wasn't
% passed in.
if nargin < 7
    optCommandGuess = repmat([grav;0],1,horizon-1); 
end

guessHorizon = size(optCommandGuess,2);
assign(optCommand(:,1:guessHorizon),optCommandGuess);

cost = 0;
constraint = [optState(:,1) == initState];

for i=1:horizon-1
    cost = cost + stepCost(optCommand(:,i), optState(:,i), goalState, Q, R);
    nextState = stepDynamics(dt, optState(:,i), optCommand(:,i), quadParam, simFlag);
    constraint = [constraint; (nextState-optState(:,i+1)) <= (1e-4)*[1;1;1;1;1;1]; nextState-optState(:,i+1) >= -(1e-4)*[1;1;1;1;1;1]];
end

% Final state cost
cost = cost + (optState(1:4,horizon)- goalState)' * F * (optState(1:4,horizon)- goalState);

% Constraint the command input
constraint = [constraint; optCommand(:,:) <= ones(2,horizon-1)*maxThrust; optCommand(:,:) >= zeros(2,horizon-1)];

%% Solve the optimization problem
options = sdpsettings('solver','fmincon','verbose', 0);
sol = optimize(constraint, cost, options);
command = double(optCommand);
state = double(optState);
end
