%MPCData Generation 
%Specify for cases that moves to right + top
%This will lead to a better result under data generation constraint
clear all;
simFlag = false;
dt = 0.02; %[s]
horizon = 8;
initPos = [0;0];
initVel = [0;0];
initPitch = 0;
initPitchRate = 0;
initState = [initPos;initVel;initPitch;initPitchRate];

% Define the cost matrix
costParam.Q = diag([2,2]);
costParam.R = diag([1000,1000,5,5]);
costParam.F = diag([2000,2000,10,10]);

quadParam.mass = 0.2; %[kg]
quadParam.Iyy = 1e-4; %[kg*m^2] Moment of inertia
quadParam.grav = 9.81; %[m/s^2]
quadParam.armLength = 0.1; %[m]
quadParam.maxThrust = 3; %[N]

% Setup of sampling 
batchNum = 20;
batchSize = 500;


posMin = -1; posMax = 2;
distMinX = 0; distMaxX = 1.5;
distMinZ = 0; distMaxZ = 1.5;
velMin = -1; velMax = 3;
pitchMin = -pi/6; pitchMax = pi/6;
pitchRateMin = -pi/6; pitchRateMax = pi;
% warning('off','all')



save('data2/ProblemSetup.mat','costParam','quadParam','horizon','dt')
for j = 1:batchNum
    stateRecord = zeros(batchSize,8); %6 element for initial state + 4 element for goal state
    commandGenerated = zeros(batchSize,(horizon-1)*2); %recording the solution of mpc
    timeRecord = zeros(batchSize,1);
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
        
       tic;
       [command, ~] = droneMPC(dt, horizon, initState, goalState, costParam, quadParam);
       duration = toc;
       commandGenerated(i,:) = reshape(command,[1,2*(horizon-1)]);
       timeRecord(i,:) = duration;
    end
    save(['data2/MPC_state',num2str(10+j),'.mat'],'stateRecord')
    save(['data2/MPC_command',num2str(10+j),'.mat'],'commandGenerated')
    save(['data2/MPC_time',num2str(10+j),'.mat'],'timeRecord')
end
warning('on','all')
