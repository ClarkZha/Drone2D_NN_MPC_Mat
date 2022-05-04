dt = 0.02; %[s]
initPos = [0;0];
initVel = [0;0];
initPitch = 0;
initPitchRate = 0;

initState = [initPos;initVel;initPitch;initPitchRate];

goalPos = [0;2];
goalVel = [1;1];
goalState = [goalPos;goalVel];

Q = diag([1,1]);
R = diag([100,100,10,10]);
F = diag([5000,5000,200,200]);

grav = 9.81; %[m/s^2]
maxAcc = 3*grav; %[m/s^2]
maxAngAcc = 100; %[rad/s^2]


horizon = 20;% Look ahead horizon for MPC
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
    [command, ~] = droneMPC(dt, horizon, currentState, goalState, Q, R, F, maxAcc, maxAngAcc);
    commandRecord(:,i) = command(:,1);
    nextState = stepDynamics(dt, currentState, command(:,1), true);
    stateRecord(:,i+1) = nextState;
    currentState = nextState;
end

plot(tRecord, stateRecord(1,:));


