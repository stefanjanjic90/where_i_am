#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

class DriveBotService {

	public:
		DriveBotService(){
			motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
			service = n.advertiseService("/ball_chaser/command_robot", &DriveBotService::handle_drive_request, this);
			ROS_INFO("Drive Bot Service ready to accept requests.");
		}

		
		bool handle_drive_request(ball_chaser::DriveToTarget::Request &req, 
															ball_chaser::DriveToTarget::Response &res){
		  
      geometry_msgs::Twist motor_command;
      motor_command.linear.x = req.linear_x;
      motor_command.angular.z = req.angular_z;
      motor_command_publisher.publish(motor_command);
		  

			res.msg_feedback = "linear_x set to " 
													+ std::to_string(req.linear_x) 
													+ " , angular_z set to: " + std::to_string(req.angular_z);

			ROS_INFO_STREAM(res.msg_feedback);

			return true;
		}

	private:
		ros::NodeHandle n;
		ros::Publisher motor_command_publisher;
		ros::ServiceServer service;
};

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

   	DriveBotService drive_bot_service;

    ros::spin();

    return 0;
}
