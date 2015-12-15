#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <p2os_msgs/MotorState.h>
#include <sensor_msgs/Joy.h>


const double translationalVel = 0.5;
const double rotationalVel = 2*M_PI/10; //rad/s
const int bufferSize = 1000;

class Joy2Vel{
private:
    ros::Publisher motorPub_;
    ros::Publisher velPub_;
    ros::Subscriber joySub_;
public:
    Joy2Vel(ros::NodeHandle& n){
        motorPub_ = n.advertise<p2os_msgs::MotorState>("/cmd_motor_state", bufferSize);
        velPub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", bufferSize);
        joySub_ = n.subscribe("/joy", bufferSize, &Joy2Vel::callback, this);
    }

    void callback(const sensor_msgs::Joy& joy ){
        if (joy.buttons[0]){
            p2os_msgs::MotorState msg;
            msg.state =1;
            motorPub_.publish(msg);
            ROS_INFO("Motors Enabled!");
        }
        else if (joy.buttons[1]){
            p2os_msgs::MotorState msg;
            msg.state = 0;
            motorPub_.publish(msg);
            ROS_INFO("Motors Disabled!");
        }

        geometry_msgs::Twist msg;
        msg.linear.x = translationalVel * joy.axes[1];
        msg.angular.z = rotationalVel * joy.axes[0];
        velPub_.publish(msg);
    }

};

int main(int argc, char** argv){
    ros::init(argc, argv, "joy2vel");
    ros::NodeHandle n;

    Joy2Vel node(n);
    ROS_INFO("Node started");
    ros::spin();
    return 0;
}
