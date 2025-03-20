#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <fstream>

class UAVInterface
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber vel_sub_;
    ros::Subscriber battery_sub_;
    ros::Publisher setpoint_pub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::TwistStamped current_vel_;
    sensor_msgs::BatteryState current_battery_;

    std::ofstream data_file_;

public:
    UAVInterface()
    {
        state_sub_ = nh_.subscribe("/mavros/state", 10, &UAVInterface::state_cb, this);
        pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &UAVInterface::pose_cb, this);
        vel_sub_ = nh_.subscribe("/mavros/local_position/velocity_local", 10, &UAVInterface::vel_cb, this);
        battery_sub_ = nh_.subscribe("/mavros/battery", 10, &UAVInterface::battery_cb, this);

        setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        data_file_.open("/tmp/gazebo_uav_data.csv");
        data_file_ << "x,y,z,vx,vy,vz,battery\n";
    }

    void state_cb(const mavros_msgs::State::ConstPtr& msg) { current_state_ = *msg; }
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) { current_pose_ = *msg; }
    void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) { current_vel_ = *msg; }
    void battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg) { current_battery_ = *msg; }

    void arm_and_offboard()
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.z = 2.0;


        for (int i = 0; i < 10; ++i)
        {
            setpoint_pub_.publish(pose);
            ros::Duration(0.1).sleep();
        }

        mavros_msgs::SetMode set_mode;
        set_mode.request.custom_mode = "OFFBOARD";
        set_mode_client_.call(set_mode);
        ROS_INFO("Switched to OFFBOARD mode");


        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        arming_client_.call(arm_cmd);
        ROS_INFO("Drone armed");
    }

    void fly_square_and_collect(int samples)
    {
        ros::Rate rate(10);
        int sample_count = 0;

        std::vector<std::vector<float>> points = {
            {0, 0, 2},
            {5, 0, 2},
            {5, 5, 2},
            {0, 5, 2},
            {0, 0, 2}
        };
        int point_idx = 0;

        while (ros::ok() && sample_count < samples)
        {
            geometry_msgs::PoseStamped target;
            target.pose.position.x = points[point_idx][0];
            target.pose.position.y = points[point_idx][1];
            target.pose.position.z = points[point_idx][2];
            setpoint_pub_.publish(target);

            float px = current_pose_.pose.position.x;
            float py = current_pose_.pose.position.y;


            if (fabs(px - points[point_idx][0]) < 0.5 && fabs(py - points[point_idx][1]) < 0.5)
            {
                point_idx = (point_idx + 1) % points.size();
            }

         
            data_file_ << px << "," << py << "," << current_pose_.pose.position.z << ","
                       << current_vel_.twist.linear.x << "," << current_vel_.twist.linear.y << "," << current_vel_.twist.linear.z << ","
                       << current_battery_.percentage << "\n";

            ++sample_count;
            ros::spinOnce();
            rate.sleep();
        }

        data_file_.close();
        ROS_INFO("Data collection complete, saved to /tmp/gazebo_uav_data.csv");
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_uav_interface");
    UAVInterface uav;
    ros::Duration(5).sleep(); 

    uav.arm_and_offboard();
    uav.fly_square_and_collect(1000);

    return 0;
}



%training interface
import pandas as pd
import torch
from torch.utils.data import Dataset

class UAVGazeboDataset(Dataset):
    def __init__(self, csv_path="/tmp/gazebo_uav_data.csv"):
        df = pd.read_csv(csv_path)
        self.X = df.iloc[:, :7].values  
        self.y = np.random.randint(0, 5, size=(len(self.X),))  

    def __len__(self):
        return len(self.X)

    def __getitem__(self, idx):
        return torch.FloatTensor(self.X[idx]), torch.LongTensor([self.y[idx]])


