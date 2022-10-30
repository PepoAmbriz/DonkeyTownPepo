#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core.hpp"
#include "iostream"

#include<cmath>


using namespace cv;
using namespace std;
/*
	Lanza el simulador:
		roslaunch bring_up navigation_inhouse.launch
	Ejecuta este nodo:
		rosrun lane_following lane_following_node
	Abrir cliente gazebo (opcional):
		gzclient
	Abrir rqt (opcional):
		rqt
	compila este (y otros) nodos:
		cd DT_PATH/ros_station_nodes/catkin_ws
		catkin_make
*/

//Añadir modelo de la cámara?
//Añadir pose (posición y orientación) de la cámara?


bool myfunction (int i,int j) { return (i<j); }

class LaneDetectorNode{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	ros::Publisher steering_pub;
	ros::Publisher speed_pub; 


public:
	static const bool filterLines = 0;

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
		const double threshold = 20.0;

		cv::Mat dst, cdst;
		
		
	 	Mat cropped_image = cv_ptr->image( Range(300,480),Range(0,640));
		cv::Canny(cropped_image, dst, 50, 200, 3);
		cv::cvtColor(dst, cdst, cv::COLOR_GRAY2BGR);
		vector<cv::Vec2f> lines;
    	cv::HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );
    	//cv::HoughLinesP(dst, lines, 1, CV_PI/180, 50, 0, 0 );
    	
    	int count = 0;
    	
    	
    	

    	for( size_t i = 0; i < lines.size(); i++){

    		count=0;

    		float rho = lines[i][0], theta = lines[i][1];
		        cv::Point pt1, pt2;
		        double a = cos(theta), b = sin(theta);
		        double x0 = a*rho, y0 = b*rho;
		        pt1.x = cvRound(x0 + 1000*(-b));
		        pt1.y = cvRound(y0 + 1000*(a));
		        pt2.x = cvRound(x0 - 1000*(-b));
		        pt2.y = cvRound(y0 - 1000*(a));
				
			
    		for( size_t j = i; j < lines.size() && count<3; j++)
    		{
    			
	        	float rhoB= lines[j][0], thetaB = lines[j][1];
		        cv::Point pt1B, pt2B;
		        double aB = cos(thetaB), bB = sin(thetaB);
		        double x0B = aB*rhoB, y0B = bB*rhoB;
		        pt1B.x = cvRound(x0B + 1000*(-bB));
		        pt1B.y = cvRound(y0B + 1000*(aB));
		        pt2B.x = cvRound(x0B - 1000*(-bB));
		        pt2B.y = cvRound(y0B - 1000*(aB));

		        double distance1 = sqrt(pow(2,pt1.x-pt1B.x) + pow(2,pt1.y-pt1B.y));
		        double distance2 = sqrt(pow(2,pt2.x-pt2B.x) + pow(2,pt2.y-pt2B.y));


		        if(distance1 < threshold && distance2 < threshold){
		        	if(filterLines){
	        			lines.erase(lines.begin() + j);
		        	}
		        	
				
					count++;
		        }
        		
    		}
    	}

		

		
		double avgx [lines.size()];
		

	    for( size_t i = 0; i < lines.size(); i++ )
    	{
	        float rho = lines[i][0], theta = lines[i][1];

	        cv::Point pt1, pt2;
	        double a = cos(theta), b = sin(theta);
	        double x0 = a*rho, y0 = b*rho;
	        pt1.x = cvRound(x0 + 500*(-b));
	        pt1.y = cvRound(y0 + 500*(a));
	        pt2.x = cvRound(x0 - 500*(-b));
	        pt2.y = cvRound(y0 - 500*(a));

	        avgx[i] = pt1.x;

	        cv::line( cdst, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
	    }
		std::vector<double> xVector (avgx, avgx+lines.size());

	    //Create path
	 	

		
		cv::imshow("Image window", cdst);
		cv::waitKey(3);
		image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char** argv){
	printf("%I <3 NYC\n");
	ros::init(argc, argv, "lane_following");
	LaneDetectorNode ldn;
	ros::spin();
	return 0;
}