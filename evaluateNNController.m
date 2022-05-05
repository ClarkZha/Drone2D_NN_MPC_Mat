modelfile = "MPC_NN.onnx";
net = importONNXLayers(modelfile, "OutputLayerType", "regression")