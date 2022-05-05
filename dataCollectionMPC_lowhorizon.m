%%MPCData Generation 

% Setup of MPC:
clear all
goalPos = [0;2];
goalVel = [1;1];
goalState = [goalPos;goalVel];

Q = diag([1,1]);
R = diag([100,100,10,10]);
F = diag([5000,5000,200,200]);

grav = 9.81; %[m/s^2]
maxAcc = 3*grav; %[m/s^2]
maxAngAcc = 100; %[rad/s^2]


% Setup of sampling 

batchNum = 20;
batchSize = 500;

dt = 0.02;
horizon = 6;

posMin = -2; posMax = 2;
distMin = -2; distMax = 2;
velMin = -4; velMax = 4;
pitchMin = -pi; pitchMax = pi;
pitchRateMin = -3*pi; pitchRateMax = 3*pi;
warning('off','all')

for j = 1:batchNum
    stateRecord = zeros(batchSize,8); %6 element for initial state + 4 element for goal state
    commandGenerated = zeros(batchSize,(horizon-1)*2); %recording the solution of mpc

    for i = 1:batchSize
       initPos = unifrnd(posMin, posMax, [2,1]);
       initVel = unifrnd(velMin, velMax, [2,1]);
       initPitch = unifrnd(pitchMin, pitchMax);
       initPitchRate = unifrnd(pitchRateMin, pitchRateMax);
       initState = [initPos; initVel; initPitch; initPitchRate];

       distX = unifrnd(distMin, distMax);
       distZ = unifrnd(distMin, distMax);
       goalPos = initPos + [distX;distZ];
       goalVel = unifrnd(velMin, velMax, [2,1]);
       goalState = [goalPos;goalVel];

       stateRecord(i,:) = [distX, distZ, initVel',goalVel',initPitch,initPitchRate];
       [command, ~] = droneMPC(dt, horizon, initState, goalState, Q, R, F, maxAcc, maxAngAcc);
       commandGenerated(i,:) = reshape(command,[1,2*(horizon-1)]);
    end
    save(['data2/MPC_state',num2str(j),'.mat'],'stateRecord')
    save(['data2/MPC_command',num2str(j),'.mat'],'commandGenerated')
end

warning('on','all')
