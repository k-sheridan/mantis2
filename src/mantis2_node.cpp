/*
 * Written by Kevin Sheridan
 */

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video.hpp"

#include "geometry_msgs/PoseArray.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include "mantis2/Mantis2Parameters.h"

#include "mantis2/Mantis2Types.h"

tf::TransformListener* tf_listener;

cv::Mat get3x3FromVector(boost::array<double, 9> vec)
{
	cv::Mat mat = cv::Mat(3, 3, CV_32F);
	for(int i = 0; i < 3; i++)
	{
		mat.at<float>(i, 0) = vec.at(3 * i + 0);
		mat.at<float>(i, 1) = vec.at(3 * i + 1);
		mat.at<float>(i, 2) = vec.at(3 * i + 2);
	}

	ROS_DEBUG_STREAM_ONCE("K = " << mat);
	return mat;
}

void callback(const sensor_msgs::ImageConstPtr& img1, const sensor_msgs::CameraInfoConstPtr& cam1, const sensor_msgs::ImageConstPtr& img2, const sensor_msgs::CameraInfoConstPtr& cam2, const sensor_msgs::ImageConstPtr& img3, const sensor_msgs::CameraInfoConstPtr& cam3)
{
	ROS_DEBUG("mantis2 start");
	Measurement measurement;

	//form all MantisImages
	measurement.img1 = MantisImage(cv_bridge::toCvCopy(img1, img1->encoding)->image, get3x3FromVector(cam1->K), img1->header.frame_id, img1->header.stamp);
	measurement.img2 = MantisImage(cv_bridge::toCvCopy(img2, img2->encoding)->image, get3x3FromVector(cam2->K), img2->header.frame_id, img2->header.stamp);
	measurement.img3 = MantisImage(cv_bridge::toCvCopy(img3, img3->encoding)->image, get3x3FromVector(cam3->K), img3->header.frame_id, img3->header.stamp);

	measurement.detectQuadrilaterals(); // find quadrilaterals in all 3 images

	ROS_DEBUG("mantis2 end");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mantis2");

	ros::NodeHandle nh;

	//setup the tf listener
	tf_listener = new tf::TransformListener();

	//setup message filter
	std::stringstream ss;
	ss.str("");
	ss << BOTTOM_CAMERA_NS << "/image_rect_color";
	message_filters::Subscriber<sensor_msgs::Image> image1_sub(nh, ss.str(), 20);
	ss.str("");
	ss << BOTTOM_CAMERA_NS << "/camera_info";
	message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo1_sub(nh, ss.str(), 20);
	ss.str("");
	ss << FRONT_CAMERA_NS << "/image_rect_color";
	message_filters::Subscriber<sensor_msgs::Image> image2_sub(nh, ss.str(), 20);
	ss.str("");
	ss << FRONT_CAMERA_NS << "/camera_info";
	message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo2_sub(nh, ss.str(), 20);
	ss.str("");
	ss << BACK_CAMERA_NS << "/image_rect_color";
	message_filters::Subscriber<sensor_msgs::Image> image3_sub(nh, ss.str(), 20);
	ss.str("");
	ss << BACK_CAMERA_NS << "/camera_info";
	message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo3_sub(nh, ss.str(), 20);


	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;

	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(50), image1_sub, cinfo1_sub, image2_sub, cinfo2_sub, image3_sub, cinfo3_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, _6));

	//start the program
	ros::spin();

}
