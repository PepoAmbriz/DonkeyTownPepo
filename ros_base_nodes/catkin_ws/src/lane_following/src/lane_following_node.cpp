#include <ros/ros.h>
#include <image_transport/image_transport.h>
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

public:
	LaneDetectorNode() : it_(nh_){
		image_sub_ = it_.subscribe("/app/camera/rgb/image_raw", 1, &LaneDetectorNode::onImg, this);
		image_pub_ = it_.advertise("/app/processed/video", 1);
		cv::namedWindow("Image window");
	}
	~LaneDetectorNode(){
		cv::destroyWindow("Image window");
	}
	void onImg(const sensor_msgs::ImageConstPtr& msg){
		cv_bridge::CvImagePtr cv_ptr; 
		try{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return; 
		}

		//Frame: cv_ptr->image

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