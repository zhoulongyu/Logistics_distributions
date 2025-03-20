# Logistics_distributions
GenAI enhanced UAV distributions with considerations of energy consumption and environmental changes
The code is based on the Flower which is suitable for imitative development with Pytorch and TensorFlow.
Here are the specific running steps:
1. Install the dependency: pip install torch flwr numpy
2. Design the model definitation
3. Generate the simulation data based on the Gazebo
4. terminal-edge UAV definitation
5. Run the federated server

Notes: you can use AirSim or Gazebo (Python API) to acquire UAV information, including positions, energy and environment information such as wind speed. 

UAV_Federated_Distillation/
├── airsim_data/
│   └── airsim_data_collector.py    
├── models/
│   └── uav_logistics_model.py       
├── datasets/
│   └── simulated_dataset.py        
├── federated/
│   ├── client.py                    
│   └── server.py                 
├── utils/
│   └── distillation_loss.py         
├── config.py                     
├── requirements.txt               
└── README.md          
