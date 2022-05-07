% Combine the generated data pack. 
dataSetNumber = 2;

X = [];
y = [];
packNum = 16;


if dataSetNumber == 1
    load('data/ProblemSetup.mat')
    for i = 1:packNum
       load(['data/MPC_state',num2str(i),'.mat'])
       load(['data/MPC_command',num2str(i),'.mat'])
       X=[X;stateRecord];
       y=[y;commandGenerated];
    end
    save('MPCdata1.mat','X','y','time','costParam','quadParam','horizon','dt');
end


if dataSetNumber == 2
    time = [];
    load('data2/ProblemSetup.mat')
    for i = 1:packNum
       load(['data2/MPC_state',num2str(i),'.mat'])
       load(['data2/MPC_command',num2str(i),'.mat'])
       load(['data2/MPC_time',num2str(i),'.mat'])
       X=[X;stateRecord];
       y=[y;commandGenerated];
       time = [time;timeRecord];
    end
    save('MPCdata2.mat','X','y','time','costParam','quadParam','horizon','dt');
end

