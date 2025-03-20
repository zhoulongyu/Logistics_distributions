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

Implementation process only for Mac/Linux:
1. chmod +x run_local_federated.sh
./run_local_federated.sh
2. build the python environment
   python -m venv venv
source venv/bin/activate
3. install the dependency
   pip install -r requirements.txt

An example: Gazebo+PX4 based data collection for GenAI training:
1. Install the Gazebo and PX4: 
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
make px4_sitl gazebo
2. Start the simulation UAV:
   make px4_sitl gazebo
3. Access the MaVROS:
   sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
roslaunch px4 mavros_posix_sitl.launch
4. ROS to collect the UAV information
5. Launch position control information through MAVROS
6. Training.





