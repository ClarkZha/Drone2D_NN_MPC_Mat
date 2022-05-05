function nextState = stepDynamics(dt, state, cmd, quadParam, simFlag)

mass = quadParam.mass;
Iyy = quadParam.Iyy;
grav = quadParam.grav;
armLength = quadParam.armLength;
maxThrust = quadParam.maxThrust;

pos = state(1:2);
vel = state(3:4);
pitch = state(5);
pitchRate = state(6);

lF = cmd(1); % Left thrust
rF = cmd(2); % Right thrust

if simFlag
    if lF > maxThrust
        lF = maxThrust;
    end
    if rF > maxThrust
        rF = maxThrust;
    end
end

thrustDir = [sin(pitch); cos(pitch)];
acc = (lF+rF)/mass * thrustDir + [0; -grav];
angAcc = (lF-rF)*armLength/Iyy;

dpos = vel * dt;
dvel = acc * dt;
dpitch = pitchRate * dt;
dpitchRate = angAcc * dt;

pos = pos+ dpos; vel = vel + dvel;
pitch = pitch + dpitch; pitchRate = pitchRate + dpitchRate;

if simFlag
    pitch = wrapToPi(pitch);
end

nextState = [pos;vel;pitch;pitchRate];
end
