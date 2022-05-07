simFlag = true;
dt = 0.02; %[s]
initPos = [0;0];
initVel = [0;0];
initPitch = 0;
initPitchRate = 0;
initState = [initPos;initVel;initPitch;initPitchRate];

goalPos = [0.5;0.5];
goalVel = [0;0];
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

horizon = 8;

T = 1; %[s] end time
n = T/dt; %Steps to simulate

stateRecord = zeros(6,n);
commandRecord = zeros(2,n);
tRecord = zeros(1,n);
stateRecord(:,1) = initState;
currentState = initState;
tRecord(1,1) = 0;
for i = 1:n-1 
    disp(tRecord(1,i));
    tRecord(1,i+1) = tRecord(1,i) + dt;
    [command, ~] = droneMPC(dt, horizon, currentState, goalState, costParam, quadParam)
    commandRecord(:,i) = command(:,1);
    nextState = stepDynamics(dt, currentState, command(:,1), quadParam, true);
    stateRecord(:,i+1) = nextState;
    currentState = nextState;
end
figure
plot(tRecord, stateRecord(1,:));
figure
plot(tRecord, stateRecord(2,:));
figure
plot(tRecord, stateRecord(3,:));
figure
plot(tRecord, stateRecord(4,:));
figure
plot(tRecord, stateRecord(5,:));




