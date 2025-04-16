# config.py

import os

class Config:
   
    RANDOM_SEED = 42
    DEVICE = 'cuda'  # 'cuda' or 'cpu'

  
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    DATA_DIR = os.path.join(BASE_DIR, "data")
    MODEL_DIR = os.path.join(BASE_DIR, "models")
    LOG_DIR = os.path.join(BASE_DIR, "logs")

   
    UAV_MAX_PAYLOAD = 5.0  # kg
    UAV_MAX_RANGE = 20.0  # km
    UAV_MAX_SPEED = 60.0  # km/h
    UAV_BATTERY_CAPACITY = 5000  # mAh

   
    AIRSIM_IP = "127.0.0.1"
    AIRSIM_PORT = 41451
    AIRSIM_DATA_INTERVAL = 1.0  # seconds

   
    GAZEBO_MODEL_STATE_TOPIC = "/gazebo/model_states"
    GAZEBO_DATA_INTERVAL = 1.0  # seconds

   
    FL_SERVER_ADDRESS = "0.0.0.0:8080"
    FL_NUM_ROUNDS = 10
    FL_CLIENT_FRACTION = 0.1
    FL_MIN_CLIENTS = 2

   
    KD_ALPHA = 0.7
    KD_TEMPERATURE = 2.0

    
    LORA_RANK = 8
    LORA_ALPHA = 32
    LORA_DROPOUT = 0.1

    
    LLAMA2_MODEL_PATH = os.path.join(MODEL_DIR, "llama2")
    LLAMA2_MAX_SEQ_LENGTH = 512

   
    BATCH_SIZE = 32
    LEARNING_RATE = 1e-4
    NUM_EPOCHS = 50

  
    LOG_INTERVAL = 10  
    SAVE_INTERVAL = 5  


def ensure_directories():
    directories = [Config.DATA_DIR, Config.MODEL_DIR, Config.LOG_DIR]
    for directory in directories:
        os.makedirs(directory, exist_ok=True)


ensure_directories()


if __name__ == "__main__":
    print("Configurations Loaded:")
    for attr in dir(Config):
        if not attr.startswith("__") and attr.isupper():
            print(f"{attr}: {getattr(Config, attr)}")
