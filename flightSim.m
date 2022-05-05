simFlag = true;
dt = 0.02; %[s]
initPos = [0;0];
initVel = [0;0];
initPitch = 0;
initPitchRate = 0;
initState = [initPos;initVel;initPitch;initPitchRate];

goalPos = [1;1];
goalVel = [1;1];
goalState = [goalPos;goalVel];

% Define the cost matrix
costParam.Q = diag([0.1,0.1]);
costParam.R = diag([100,100,10,10]);
costParam.F = diag([5000,5000,200,200]);

quadParam.mass = 0.2; %[kg]
quadParam.Iyy = 1e-4; %[kg*m^2] Moment of inertia
quadParam.grav = 9.81; %[m/s^2]
quadParam.armLength = 0.1; %[m]
quadParam.maxThrust = 4; %[N]

horizon = 10;

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
    [command, ~] = droneMPC(dt, horizon, initState, goalState, costParam, quadParam)
    commandRecord(:,i) = command(:,1);
    nextState = stepDynamics(dt, currentState, command(:,1), quadParam, true);
    stateRecord(:,i+1) = nextState;
    currentState = nextState;
end

plot(tRecord, stateRecord(1,:));
plot(tRecord, stateRecord(2,:));
plot(tRecord, stateRecord(3,:));
plot(tRecord, stateRecord(4,:));
plot(tRecord, stateRecord(5,:));




