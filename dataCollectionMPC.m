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
n = 10; %Number of data to collect
dt = 0.02;
horizon = 15;
stateRecord = zeros(n,10); %6 element for initial state + 4 element for goal state
commandGenerated = zeros(n,(horizon-1)*2); %recording the solution of mpc

posMin = -2; posMax = 2;
distMin = -3; distMax = 3;
velMin = -4; velMax = 4;
pitchMin = -pi; pitchMax = pi;
pitchRateMin = -3*pi; pitchRateMax = 3*pi;

warning('off','all')

for i = 1:n
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
   
   stateRecord(i,:) = [initState',goalState'];
   [command, ~] = droneMPC(dt, horizon, initState, goalState, Q, R, F, maxAcc, maxAngAcc);
   commandGenerated(i,:) = reshape(command,[1,2*(horizon-1)]);
end
save('MPC_input.mat','stateRecord')
save('MPC_command.mat','commandGenerated')

warning('on','all')
