function nextState = stepDynamics(dt, state, cmd, wrap)

pos = state(1:2);
vel = state(3:4);
pitch = state(5);
pitchRate = state(6);
cmdAcc = cmd(1);
cmdAngAcc = cmd(2);

grav = 9.81;

thrustDir = [sin(pitch); cos(pitch)];
acc = cmdAcc * thrustDir + [0; -grav];

dpos = vel * dt;
dvel = acc * dt;
dpitch = pitchRate * dt;
dpitchRate = cmdAngAcc*dt;

pos = pos+ dpos; vel = vel + dvel;
pitch = pitch + dpitch; pitchRate = pitchRate + dpitchRate;

if wrap
    pitch = wrapToPi(pitch);
end

nextState = [pos;vel;pitch;pitchRate];

end
