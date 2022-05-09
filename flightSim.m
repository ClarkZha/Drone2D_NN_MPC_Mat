dataSetNumber = 2;

if dataSetNumber == 1
    load('TrainingResult1.mat')
    actionHorizon = 4; %Output horizon of the NN controller
elseif dataSetNumber == 2
    load('TrainingResult2.mat')
    actionHorizon = 2; %Output horizon of the NN controller
elseif dataSetNumber == 3
    load('TrainingResult3.mat')
    actionHorizon = 4; %Output horizon of the NN controller
end 

simFlag = true;
dt = 0.02; %[s]
initPos = [0;0];
initVel = [0;0];
initPitch = 0;
initPitchRate = 0;
initState = [initPos;initVel;initPitch;initPitchRate];

%goalPos = [0.2;1.0];

% goalPos = [0.1;1.0];
% goalVel = [0.1;1.0];

goalPos = [0.2;1.0];
goalVel = [0.0;0.0];

goalState = [goalPos;goalVel];

% Define the cost matrix
costParam.Q = diag([2,2]);
costParam.R = diag([1000,1000,5,5]);
costParam.F = diag([2000,2000,10,10]);

quadParam.mass = 0.2; %[kg]
quadParam.Iyy = 1e-4; %[kg*m^2] Moment of inertia
quadParam.grav = 9.81; %[m/s^2]
quadParam.armLength = 0.1; %[m]
quadParam.maxThrust = 3; %[N]

stateSize = 8; %Size of input state
actionSize = 2; %Use 2 for other ones


horizon = 8;

T = 0.3; %[s] end time
n = T/dt; %Steps to simulate

stateRecord_MPC = zeros(6,n);
stateRecord_NN = zeros(6,n);
stateRecord_NN_MPC = zeros(6,n);
commandRecord_MPC = zeros(2,n);
commandRecord_NN = zeros(2,n);
commandRecord_NN_MPC = zeros(2,n);


% Simulate with MPC controller
tRecord = zeros(1,n);
tRecord(1,1) = 0;
stateRecord_MPC(:,1) = initState;
currentState = initState;


for i = 1:n-1 
    disp(tRecord(1,i));
    tRecord(1,i+1) = tRecord(1,i) + dt;
    [command, ~] = droneMPC(dt, horizon, currentState, goalState, costParam, quadParam);
    commandRecord_MPC(:,i) = command(:,1);
    nextState = stepDynamics(dt, currentState, command(:,1), quadParam, true);
    stateRecord_MPC(:,i+1) = nextState;
    currentState = nextState;
end


% Simulate with NN controller
stateRecord_NN(:,1) = initState;
currentState = initState;
for i = 1:n-1 
    distX = goalState(1) - currentState(1);
    distZ = goalState(2) - currentState(2);
    initVel = currentState(3:4);
    goalVel = goalState(3:4);
    initPitch = currentState(5);
    initPitchRate = currentState(6);
    predInput = [distX, distZ, initVel',goalVel',initPitch,initPitchRate];
    NN_output = predict(MPCNetObj, predInput);
    NN_command = double(reshape(NN_output, [actionSize,actionHorizon]));
    commandRecord_NN(:,i) = NN_command(:,1);
    nextState = stepDynamics(dt, currentState, NN_command(:,1), quadParam, true);
    stateRecord_NN(:,i+1) = nextState;
    currentState = nextState;
end


% Simulate with NN-MPC controller
stateRecord_NN_MPC(:,1) = initState;
currentState = initState;
for i = 1:n-1 
    distX = goalState(1) - currentState(1);
    distZ = goalState(2) - currentState(2);
    initVel = currentState(3:4);
    goalVel = goalState(3:4);
    initPitch = currentState(5);
    initPitchRate = currentState(6);
    predInput = [distX, distZ, initVel',goalVel',initPitch,initPitchRate];
    NN_output = predict(MPCNetObj, predInput);
    NN_command = double(reshape(NN_output, [actionSize,actionHorizon]));
    [NN_MPC_command, ~] = droneMPC(dt, horizon, initState, goalState, costParam, quadParam, NN_command);   
    commandRecord_NN_MPC(:,i) = NN_MPC_command(:,1);
    nextState = stepDynamics(dt, currentState, NN_MPC_command(:,1), quadParam, true);
    stateRecord_NN_MPC(:,i+1) = nextState;
    currentState = nextState;
end

figure
subplot(3,2,1)
plot(tRecord, stateRecord_MPC(1,:),'Color','#0072BD','LineWidth',2);
hold on
plot(tRecord, stateRecord_NN(1,:),'-o');
plot(tRecord, stateRecord_NN_MPC(1,:),'-s');
h = xlabel('t [s]','FontSize',10);
d = ylabel('pos-x [m]','FontSize',10);
AX=legend('MPC', 'NN', 'NN-MPC', 'location','northwest');

subplot(3,2,2)
plot(tRecord, stateRecord_MPC(2,:),'Color','#0072BD','LineWidth',2);
hold on
plot(tRecord, stateRecord_NN(2,:),'-o');
plot(tRecord, stateRecord_NN_MPC(2,:),'-s');
xlabel('t [s]','FontSize',10);
ylabel('pos-z [m]','FontSize',10);


subplot(3,2,3)
plot(tRecord, stateRecord_MPC(3,:),'Color','#0072BD','LineWidth',2);
hold on
plot(tRecord, stateRecord_NN(3,:),'-o');
plot(tRecord, stateRecord_NN_MPC(3,:),'-s');
h = xlabel('t [s]','FontSize',10);
d =ylabel('vel-x [m/s]','FontSize',10);

subplot(3,2,4)
plot(tRecord, stateRecord_MPC(4,:),'Color','#0072BD','LineWidth',2);
hold on
plot(tRecord, stateRecord_NN(4,:),'-o');
plot(tRecord, stateRecord_NN_MPC(4,:),'-s');
h = xlabel('t [s]','FontSize',10);
d = ylabel('vel-z [m/s]','FontSize',10);


subplot(3,2,5)
plot(tRecord, stateRecord_MPC(5,:),'Color','#0072BD','LineWidth',2);
hold on
h = plot(tRecord, stateRecord_NN(5,:),'-o');
d = plot(tRecord, stateRecord_NN_MPC(5,:),'-s');
xlabel('t [s]','FontSize',10);
ylabel('pitch [rad]','FontSize',10);

subplot(3,2,6)
plot(tRecord, stateRecord_MPC(6,:),'Color','#0072BD','LineWidth',2);
hold on
h = plot(tRecord, stateRecord_NN(6,:),'-o');
d = plot(tRecord, stateRecord_NN_MPC(6,:),'-s');
xlabel('t [s]','FontSize',10);
ylabel('pitch-rate [rad/s]','FontSize',10);


save('simResult.mat','stateRecord_MPC','stateRecord_NN','stateRecord_NN_MPC')
