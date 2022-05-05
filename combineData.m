packNum = 9;

X = [];
y = [];

for i = 1:packNum
   load(['data/MPC_state',num2str(i),'.mat'])
   load(['data/MPC_command',num2str(i),'.mat'])
   X=[X;stateRecord];
   y=[y;commandGenerated];
end

save(['MPCdata.mat'],'X','y');