# Imports for pytorch
import numpy as np
import torch
from torch import nn, optim
from matplotlib import pyplot as plt
import torch.nn as nn
import torch.nn.functional as F
from scipy.io import loadmat
import torch.onnx



mpc_data = loadmat('MPCdata.mat')
X = mpc_data['X']
y = mpc_data['y']

# Train-test split
train_number = int(0.8*X.shape[0])
test_number = X.shape[0] - train_number
output_horizon = 3 # Output horizon of the neural net controller
command_dim = 2 #dim of each command
loader_batch_size = 100

train_loader = torch.utils.data.DataLoader([[X[i,:], y[i,:command_dim*output_horizon]]\
     for i in range(train_number)], shuffle=True, batch_size=loader_batch_size)

test_loader = torch.utils.data.DataLoader([[X[i+train_number,:], \
    y[i+train_number,:command_dim*output_horizon]]\
        for i in range(test_number)], shuffle=True, batch_size=loader_batch_size)

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Define neural net
class MPC_Net(nn.Module):
  def __init__(self,input_dim, output_dim):
    super(MPC_Net, self).__init__()
    self.fc1 = nn.Linear(input_dim, 64)
    self.fc2 = nn.Linear(64, 64)
    self.fc3 = nn.Linear(64, 32)
    self.fc4 = nn.Linear(32, output_dim)

  def forward(self, x):
    x = F.relu(self.fc1(x))
    x = F.relu(self.fc2(x))
    x = F.relu(self.fc3(x))
    x = self.fc4(x)
    return x



net = MPC_Net(X.shape[1],output_horizon*command_dim)
net.to(device)
loss_function = nn.MSELoss()
optimizer = optim.SGD(net.parameters(), lr=0.001, momentum=0.8)


epochs = 15 
training_loss_list =[]
validation_loss_list =[]

for epoch in range(epochs):  # loop over the dataset multiple times
    running_loss = 0.0
    epoch_loss = 0.0
    for i, data in enumerate(train_loader, 0):
        state, command = data[0].float().to(device), data[1].float().to(device)
        optimizer.zero_grad()
        outputs = net.forward(state)
        loss = loss_function(outputs, command)
        loss.backward()
        optimizer.step()

        # print statistics
        running_loss += loss.item()
        epoch_loss += loss.item()
        if i % 5 == 4:    # print every 5 mini-batches
            print(f'[{epoch + 1}, {i + 1:5d}] loss: {running_loss / 500:.3f}')
            running_loss = 0.0
    training_loss_list.append(epoch_loss/len(train_loader))
    
    # Record validation loss
    # It will be commented out when runnig the final training process for kaggle

    with torch.no_grad():
        validation_loss = 0
        for data in test_loader:
            state, command = data[0].float().to(device), data[1].float().to(device)
            # calculate outputs by running images through the network
            outputs = net(state)
            validation_loss += loss_function(outputs, command).item()
    validation_loss_list.append(validation_loss/len(test_loader))


fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.plot(range(len(training_loss_list)), training_loss_list, label = 'Training Loss')
ax.plot(range(len(validation_loss_list)), validation_loss_list, label = 'Validation Loss')
ax.set_ylabel('Loss')
ax.set_xlabel('Epoch')
ax.legend()
ax.set_title('MPC_NN: Loss VS Epoch')

net.to('cpu')


net.eval()

# Save the trained network as an onnx file so it can be imported and run in matlab
dummy_input = torch.randn(1, 8)
input_names = [ "actual_input" ]
output_names = [ "output" ]
torch.onnx.export(net, 
                  dummy_input,
                  "MPC_NN.onnx",
                  verbose=False,
                  input_names=input_names,
                  output_names=['output_1','output_2','output_3','output_4','output_5','output_6'],
                  export_params=True,
                  )
plt.show()



