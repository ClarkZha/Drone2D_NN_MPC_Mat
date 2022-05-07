%% Unit test
% Use a basic example to check if the MPC pipeline is working 
% CZ 2022

dt = 0.02; %[s]
initPos = [0;0];
initVel = [0;0];
initPitch = 0;
initPitchRate = 0;

initState = [initPos;initVel;initPitch;initPitchRate];

goalPos = [1;2];
goalVel = [0.5;0.5];
goalState = [goalPos;goalVel];

% Define the cost matrix
costParam.Q = diag([2,2]);
costParam.R = diag([100,100,5,5]);
costParam.F = diag([200,200,10,10]);


quadParam.mass = 0.2; %[kg]
quadParam.Iyy = 1e-4; %[kg*m^2] Moment of inertia
quadParam.grav = 9.81; %[m/s^2]
quadParam.armLength = 0.1; %[m]
quadParam.maxThrust = 3; %[N]

horizon = 8;
[command, state] = droneMPC(dt, horizon, initState, goalState, costParam, quadParam)