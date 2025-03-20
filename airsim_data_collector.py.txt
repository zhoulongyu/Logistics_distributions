import airsim
import numpy as np

class AirSimDataCollector:
    def __init__(self):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

    def collect_data(self, num_samples=1000):
        X, y = [], []
        for _ in range(num_samples):
            state = self.client.getMultirotorState()
            pos = state.kinematics_estimated.position
            battery = np.random.uniform(0.5, 1.0)  # 电量
            weather = np.random.uniform(0, 1, 3)  # 环境参数
            feature = [pos.x_val, pos.y_val, pos.z_val, battery] + list(weather)
            X.append(feature)
            y.append(np.random.randint(0, 5))  # 假设5个配送决策
        return np.array(X), np.array(y)
