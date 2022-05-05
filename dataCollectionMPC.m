%%MPCData Generation 
clear all;


% Define the cost matrix
costParam.Q = diag([0.1,0.1]);
costParam.R = diag([100,100,10,10]);
costParam.F = diag([5000,5000,200,200]);


quadParam.mass = 0.2; %[kg]
quadParam.Iyy = 1e-4; %[kg*m^2] Moment of inertia
quadParam.grav = 9.81; %[m/s^2]
quadParam.armLength = 0.1; %[m]
quadParam.maxThrust = 3; %[N]

% Setup of sampling 

batchNum = 20;
batchSize = 500;

dt = 0.02;
horizon = 5;

posMin = -2; posMax = 2;
distMinX = -1.5; distMaxX = 1.5;
distMinZ = 0; distMaxZ = 1.5;
velMin = -3; velMax = 3;
pitchMin = -pi/6; pitchMax = pi/6;
pitchRateMin = -pi; pitchRateMax = pi;
% warning('off','all')

for j = 1:batchNum
    stateRecord = zeros(batchSize,8); %6 element for initial state + 4 element for goal state
    commandGenerated = zeros(batchSize,(horizon-1)*2); %recording the solution of mpc

    for i = 1:batchSize
       initPos = unifrnd(posMin, posMax, [2,1]);
       initVel = unifrnd(velMin, velMax, [2,1]);
       initPitch = unifrnd(pitchMin, pitchMax);
       initPitchRate = unifrnd(pitchRateMin, pitchRateMax);
       initState = [initPos; initVel; initPitch; initPitchRate];

       distX = unifrnd(distMinX, distMaxX);
       distZ = unifrnd(distMinZ, distMaxZ);
       goalPos = initPos + [distX;distZ];
       goalVel = unifrnd(velMin, velMax, [2,1]);
       goalState = [goalPos;goalVel];

       stateRecord(i,:) = [distX, distZ, initVel',goalVel',initPitch,initPitchRate];

       [command, ~] = droneMPC(dt, horizon, initState, goalState, costParam, quadParam);
       commandGenerated(i,:) = reshape(command,[1,2*(horizon-1)]);
    end
    save(['data/MPC_state',num2str(j),'.mat'],'stateRecord')
    save(['data/MPC_command',num2str(j),'.mat'],'commandGenerated')
end

warning('on','all')
