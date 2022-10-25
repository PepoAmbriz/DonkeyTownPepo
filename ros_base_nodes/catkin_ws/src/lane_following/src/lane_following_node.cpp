#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Añadir modelo de la cámara?
//Añadir pose (posición y orientación) de la cámara?

class LaneDetectorNode{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	ros::Publisher steering_pub;
	ros::Publisher speed_pub; 

public:
	LaneDetectorNode() : it_(nh_){
		image_sub_ = it_.subscribe("/app/camera/rgb/image_raw", 1, &LaneDetectorNode::onImg, this);
		image_pub_ = it_.advertise("/app/processed/video", 1);
		steering_pub = nh_.advertise<std_msgs::Int16>("/AutoModelMini/manual_control/steering", 1);
		speed_pub = nh_.advertise<std_msgs::Int16>("/AutoModelMini/manual_control/speed", 1);
		cv::namedWindow("Image window");
	}
	~LaneDetectorNode(){
		cv::destroyWindow("Image window");
	}
	void publish_steering(float steering){
		std_msgs::Int16 steering_msg;
		steering_msg.data = 90*(steering+1);
		steering_pub.publish(steering_msg);
	}
	void publish_speed(float speed){
		std_msgs::Int16 speed_msg;
		speed_msg.data = -826.66*speed;
		speed_pub.publish(speed_msg);
	}
	void stop(void){
		this->publish_speed(0.0);
	}
	//Main callback
	//Receives img from simulation. 
	//	"cv_ptr->image" is an OpenCV Mat object.
	//	publish_speed(float speed); updates car's speed in m/s
	// 	pubilsh_steering(float steering); updates car's steering in degrees (from -90° to 90°)
	void onImg(const sensor_msgs::ImageConstPtr& msg){
		cv_bridge::CvImagePtr cv_ptr; 
		try{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return; 
		}

		//Frame: cv_ptr->image
		/*
		if(ros::ok()){
			this->publish_speed(<speed_value>);
			this->publish_steering(<steering_value>);
		}
		*/
		cv::imshow("Image window", cv_ptr->image);
		cv::waitKey(3);
		image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char** argv){
	ros::init(argc, argv, "lane_following");
	LaneDetectorNode ldn;
	ros::spin();
	return 0;
}