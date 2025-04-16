import rospy
from gazebo_msgs.msg import ModelStates
import os
import numpy as np
import time

class GazeboDataCollector:
    def __init__(self, save_path="gazebo_data"):
        self.save_path = save_path
        os.makedirs(self.save_path, exist_ok=True)
        rospy.init_node('gazebo_data_collector', anonymous=True)
        self.subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        self.data = []

    def callback(self, data):
        timestamp = time.time()
        for i, name in enumerate(data.name):
            if "uav" in name:
                position = data.pose[i].position
                velocity = data.twist[i].linear
                self.data.append({
                    "timestamp": timestamp,
                    "name": name,
                    "position": {
                        "x": position.x,
                        "y": position.y,
                        "z": position.z
                    },
                    "velocity": {
                        "x": velocity.x,
                        "y": velocity.y,
                        "z": velocity.z
                    }
                })

    def save_data(self):
        filename = os.path.join(self.save_path, f"{int(time.time())}.npy")
        np.save(filename, self.data)

if __name__ == "__main__":
    collector = GazeboDataCollector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        collector.save_data()
