#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <jackal_controller/StampedModelStates.h>

ros::Publisher stamped_model_states_pub;

void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    jackal_controller::StampedModelStates stamped_msg;
    stamped_msg.header.stamp = ros::Time::now();
    stamped_msg.header.frame_id = "world";

    stamped_msg.name = msg->name;
    stamped_msg.pose = msg->pose;
    stamped_msg.twist = msg->twist;

    stamped_model_states_pub.publish(stamped_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "model_state_stamper");
    ros::NodeHandle nh;

    ros::Subscriber model_states_sub = nh.subscribe("/gazebo/model_states", 10, modelStatesCallback);
    stamped_model_states_pub = nh.advertise<jackal_controller::StampedModelStates>("/gazebo/stamped_model_states", 10);

    ros::spin();
    return 0;
}

