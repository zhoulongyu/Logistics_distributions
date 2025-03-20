from torch.utils.data import Dataset
import numpy as np
import torch

class UAVSimulatedDataset(Dataset):
    def __init__(self, num_samples=2000, region_factor=1.0):
        self.X = np.random.rand(num_samples, 7) * region_factor
        self.y = np.random.randint(0, 5, size=(num_samples,))

    def __len__(self):
        return len(self.X)

    def __getitem__(self, idx):
        return torch.FloatTensor(self.X[idx]), torch.LongTensor([self.y[idx]])
