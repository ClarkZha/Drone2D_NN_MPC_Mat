clear all
load('MPCdata.mat')



XTrain = arrayDatastore(X);
yTrian = arrayDatastore(y);

predictHorizon = 3;
outDim = 2;
hiddenLayerSize = [64;64;128;128;64];
outputSize = predictHorizon*outDim;
numState = 8;

MPCNet = [
%     featureInputLayer(numState,'Normalization','none','Name','observation')

    fullyConnectedLayer(numState,'Name','fc0')
    reluLayer('Name','relu0')
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
    fullyConnectedLayer(outputSize,'Name','fcLast')
%     tanhLayer('Name','tanhLast')
%     scalingLayer('Name','ActorScaling','Scale',umax)
    regressionLayer('Name','routput')];

net = dlnetwork(layers)
