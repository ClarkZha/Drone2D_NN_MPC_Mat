% Shuffle and split the data into training, validation and test sections
function prepedData  = prepareData(dataSet)
    % Unpack the data
    stateSize = dataSet.stateSize; %Size of input state
    actionSize = dataSet.actionSize;
    actionHorizon = dataSet.actionHorizon;
    data =  [dataSet.X, dataSet.y];
    validatePercent = dataSet.validatePercent;
    testPercent = dataSet.testPercent;
    
    % Decide split number 
    N = size(data,1);
    validateN = floor(validatePercent*N);
    testN = floor(testPercent*N);

    
    % create validation and training data
    ir=randperm(size(dataSet.y,1)); % since y is 1D; otherwise use size(y,1)
    randomData=data(ir,:);

    prepedData.TrainInput = randomData(1:N-validateN-testN,1:stateSize);
    prepedData.TrainOutput = randomData(1:N-validateN-testN,stateSize+1:stateSize + actionSize * actionHorizon);

    validateInput = randomData(N-validateN-testN+1:N-testN,1:stateSize);
    validateOutput = randomData(N-validateN-testN+1:N-testN,stateSize+1:stateSize+actionSize*actionHorizon);
    prepedData.validateCellArray = {validateInput, validateOutput};
    
    prepedData.testInput = randomData(N-testN+1:N, 1:stateSize);
    prepedData.testOutput = randomData(N-testN+1:N, stateSize+1:stateSize+actionSize*actionHorizon);
end