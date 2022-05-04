
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
maxAngAcc = 200; %[rad/s^2]

horizon = 30;

[command, state] = droneMPC(dt, horizon, initState, goalState, Q, R, F, maxAcc, maxAngAcc)