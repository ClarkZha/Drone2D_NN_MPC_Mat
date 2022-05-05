% Train the MPC_NN from the MPC data generated.
clear all
load('MPCdata.mat')

maxThrust = 3; %[N]
XTrain = X;
yTrian = y;

dataSet.stateSize = 8; %Size of input state
dataSet.actionSize = 2;
dataSet.actionHorizon = 2;
dataSet.X = X;
dataSet.y = y;
dataSet.validatePercent = 0.05; %Use 10% data to validate
dataSet.testPercent = 0.15; %Use 15% data to test
prepedData  = prepareData(dataSet);

hiddenLayerSize = [64;64;128;128;64];
intSize = dataSet.stateSize;
outSize = dataSet.actionSize*dataSet.actionHorizon;

% Setup the network, fully connected layers with ReLU in between
MPCNet = [
    featureInputLayer(intSize,'Normalization','none','Name','observation')
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
    'MiniBatchSize', 100, ...
    'ValidationData', prepedData.validateCellArray, ...
    'InitialLearnRate', 1e-3, ...
    'ExecutionEnvironment', 'cpu', ...
    'GradientThreshold', 10, ...
    'MaxEpochs', 30 ...
    );

MPCNetObj = trainNetwork(prepedData.TrainInput,prepedData.TrainOutput,MPCNet,options);

% Test Network
predictOutput = predict(MPCNetObj,prepedData.testInput);

% RMSE calculation
testError = sqrt(mean((prepedData.testOutput - predictOutput).^2));
fprintf('Final Test RMS Error = %d\n', testError);