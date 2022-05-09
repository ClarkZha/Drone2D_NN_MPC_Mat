% Train the MPC_NN from the MPC data generated.
clear all
%% Prepare trianing and validation data
% Choose which dataset to train on
dataSetNumber = 2;
% Load the training data and parameters.
if dataSetNumber == 1
    load('MPCdata1.mat')
elseif dataSetNumber == 2
    load('MPCdata2.mat')
elseif dataSetNumber == 3
    load('MPCdata3.mat')
end 


maxThrust = 3; %[N]
XTrain = X;
yTrian = y;

dataSet.stateSize = 8; %Size of input state
dataSet.actionSize = 2;
dataSet.actionHorizon = 2; %Output horizon of the NN controller
dataSet.X = X;
dataSet.y = y;

if exist('time','var')
    %The data set has recorded MPC computation time
    dataSet.time = time;
    timeRecordFlag = true; 
else
    %The data set has no MPC computation time
    dataSet.time = zeros(size(y,1),1);
    timeRecordFlag = false;
end 

dataSet.validatePercent = 0.05; %Use 5% data to validate
dataSet.testPercent = 0.10; %Use 10% data to test
prepedData  = prepareData(dataSet);



%% Train the neural net
hiddenLayerSize = [64;128;256;128;64];
inSize = dataSet.stateSize;
outSize = dataSet.actionSize*dataSet.actionHorizon;

epochNum = 30;
batchSize = 100;

% Setup the network, fully connected layers with ReLU in between
MPCNet = [
    featureInputLayer(inSize,'Normalization','none','Name','observation')
    fullyConnectedLayer(hiddenLayerSize(1),'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(hiddenLayerSize(2),'Name','fc2')
    reluLayer('Name','relu2')
    fullyConnectedLayer(hiddenLayerSize(3),'Name','fc3')
    reluLayer('Name','relu3')
    fullyConnectedLayer(hiddenLayerSize(4),'Name','fc4')
    reluLayer('Name','relu4')
    fullyConnectedLayer(hiddenLayerSize(5),'Name','fc5')
    reluLayer('Name','relu5')
    fullyConnectedLayer(outSize,'Name','fcLast')
    tanhLayer('Name','tanh') %
    scalingLayer('Name','Scaling','Scale',maxThrust)
    regressionLayer('Name','routput')];

options = trainingOptions('adam', ...
    'Verbose', false, ...
    'Plots', 'training-progress', ...
    'Shuffle', 'every-epoch', ...
    'MiniBatchSize', batchSize, ...
    'ValidationData', prepedData.validateCellArray, ...
    'ValidationFrequency',1, ...
    'InitialLearnRate', 1e-3, ...
    'ExecutionEnvironment', 'cpu', ...
    'GradientThreshold', 10, ...
    'MaxEpochs', epochNum ...
    );

[MPCNetObj, trainInfo] = trainNetwork(prepedData.TrainInput,prepedData.TrainOutput,MPCNet,options);

%% Plot the training/validation data

epochSampleSize = floor(size(prepedData.TrainInput,1)/batchSize);
epochLossList = zeros(2,epochNum);
for i = 1:epochNum
    epochLossList(1,i) = trainInfo.TrainingLoss(1,epochSampleSize*i);
    epochLossList(2,i) = trainInfo.ValidationLoss(1,epochSampleSize*i);
end


figure
plot(1:epochNum,epochLossList(1,:),'Color','#0072BD','LineWidth',2)
hold on
plot(1:epochNum,epochLossList(2,:),'Color','#D95319','LineWidth',2)
legend('Training','Validation')
xlabel('Epoch')
ylabel('Loss')

% RMSE calculation
predictOutput = predict(MPCNetObj,prepedData.testInput);
testError = sqrt(mean((prepedData.testOutput - predictOutput).^2));
fprintf('Final Test RMS Error = %d\n', testError);



%%  Result evaluation 
% We compare the performance and computation time of 
% 1. The original MPC controller
% 2. The NN controller
% 3. The MPC controller initiated by 
testSize = size(prepedData.testInput,1);
computationTimeList = zeros(testSize,3);
costList = zeros(testSize,3);
for i = 1:testSize
    % Unload the problem setup:
    distX =  prepedData.testInput(i,1);
    distZ =  prepedData.testInput(i,2);
    initVel = prepedData.testInput(i,3:4)';
    goalVel = prepedData.testInput(i,5:6)';
    initPitch =  prepedData.testInput(i,7);
    initPitchRate = prepedData.testInput(i,8);
    initState = [0;0;initVel;initPitch;initPitchRate];
    goalState = [distX;distZ;goalVel];

    % Original MPC Controller
    commandMPC = reshape(prepedData.testOutput(i,:),[dataSet.actionSize,dataSet.actionHorizon]);
    
    if timeRecordFlag
        % Use the original solver time if it is recorded
        computationTimeList(i,1) = prepedData.time(i,:);
    else
        % Solve the MPC and record the computation time
        tic;
        [commandMPC, ~] = droneMPC(dt, horizon, initState, goalState, costParam, quadParam);
        computationTimeList(i,1) = toc;
    end
    costList(i,1) = computeCostRollOut(dt, dataSet.actionHorizon, initState, goalState, commandMPC, costParam, quadParam);

    % NN controller
    tic;
    commandNN = double(reshape(predict(MPCNetObj, prepedData.testInput(i,:)),[dataSet.actionSize,dataSet.actionHorizon]));
    computationTimeList(i,2) = toc;
    commandNN = reshape(commandNN, [dataSet.actionSize,dataSet.actionHorizon]);
    costList(i,2) = computeCostRollOut(dt, dataSet.actionHorizon, initState, goalState, commandNN, costParam, quadParam);

    % MPC primed with NN result. 
    tic;
    [commandNNMPC, ~] = droneMPC(dt, horizon, initState, goalState, costParam, quadParam, commandNN);
    duration = toc; 
    computationTimeList(i,3) = toc;
    costList(i,3) = computeCostRollOut(dt, dataSet.actionHorizon, initState, goalState, commandNNMPC, costParam, quadParam);
end

sScoreNN = ones(testSize,1) - abs(costList(:,2) - costList(:,1))./costList(:,1);
sScoreNN_MPC = ones(testSize,1) - abs(costList(:,3) - costList(:,1))./costList(:,1);

if dataSetNumber == 1
    save('TrainingResult1','computationTimeList','costList','epochLossList','MPCNetObj')
elseif dataSetNumber == 2
    save('TrainingResult2','computationTimeList','costList','epochLossList','MPCNetObj')
elseif dataSetNumber == 3
    save('TrainingResult3','computationTimeList','costList','epochLossList','MPCNetObj')
end 

fprintf('mean CompTime = %d\n', mean(computationTimeList));
fprintf('var CompTime = %d\n', var(computationTimeList));

fprintf('mean S-score NN = %d\n', mean(sScoreNN));
fprintf('var S-score NN= %d\n', var(sScoreNN));

fprintf('mean S-score NN_MPC = %d\n', mean(sScoreNN_MPC));
fprintf('var S-score NN_MPC= %d\n', var(sScoreNN_MPC));

