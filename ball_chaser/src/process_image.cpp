#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

ros::ServiceClient client;

void drive_robot(float lin_x, float ang_z)
{
		ROS_INFO("Requesting to drive robot.");
		ball_chaser::DriveToTarget drive_to_target;
		drive_to_target.request.linear_x = lin_x;
		drive_to_target.request.angular_z = ang_z;

		if (!client.call(drive_to_target)){
			ROS_ERROR("Failed to call service command robot.");
		}
}

void process_image_callback(const sensor_msgs::Image image){
	int white_pixel = 255;

	int sector_length = image.step / 3;
	int left_end = sector_length - 1;
	int mid_start = left_end + 1;
	int mid_end = mid_start + sector_length;
	int right_start = mid_end + 1;

	bool left = false;
	bool right = false;
	bool mid = false;

	for(int i = 0; i < image.height * image.step; i++){
		int pixel_location = i % image.step;
		if (pixel_location <= left_end && image.data[i] == white_pixel){
			left = true;
			right = false;
			mid = false;
			break;
		}else if(pixel_location >= mid_start 
							&& pixel_location <= mid_end
							 && image.data[i] == white_pixel){
			left = false;
			right = false;
			mid = true;
			break;
		}else if(pixel_location >= right_start && image.data[i] == white_pixel){
			left = false;
			right = true;
			mid = false;
			break;
		}
	}

	if(left){
		ROS_INFO("DRIVING LEFT");
		drive_robot(0.01, 0.15);
	}else if(right){
		ROS_INFO("DRIVING RIGHT");
		drive_robot(0.01, -0.15);
	}else if (mid){
		ROS_INFO("DRIVING MID");
		drive_robot(0.15, 0.0);
	}else{
		ROS_INFO("DRIVING STOP");
		drive_robot(0.0, 0.0);
	}
}

int main(int argc, char** argv)
{
    ROS_INFO("Initializing Process Image Node.");

    ros::init(argc, argv, "process_image");

		ros::NodeHandle n;

		client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

		ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    ros::spin();

    return 0;
}
