import torch.nn as nn
import torch.nn.functional as F

class UAVLogisticsModel(nn.Module):
    def __init__(self, input_dim=7, hidden_dim=512, output_dim=5):
        super(UAVLogisticsModel, self).__init__()
        self.fc1 = nn.Linear(input_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, output_dim)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        return self.fc3(x)
