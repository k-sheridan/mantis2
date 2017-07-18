/*
 * Mantis3Types.h
 *
 *  Created on: Jun 11, 2017
 *      Author: pauvsi
 */

#ifndef MANTIS_INCLUDE_MANTIS3_MANTIS3TYPES_H_
#define MANTIS_INCLUDE_MANTIS3_MANTIS3TYPES_H_


#include <iosfwd>
#include <ostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

#include "Mantis2Parameters.h"

/*
 * a guess about our pose
 */
struct Hypothesis{
private:
	tf::Transform c2w, w2c; // cam to/from world transform
	tf::Quaternion q;
public:
	double error;
	int observations;

	geometry_msgs::PoseStamped toPoseMsg(ros::Time stamp, std::string frame)
	{
		geometry_msgs::PoseStamped msg;

		tf::Vector3 pos = w2c.getOrigin();

		msg.pose.position.x = pos.x();
		msg.pose.position.y = pos.y();
		msg.pose.position.z = pos.z();

		msg.pose.orientation.w = q.w();
		msg.pose.orientation.x = q.x();
		msg.pose.orientation.y = q.y();
		msg.pose.orientation.z = q.z();

		msg.header.frame_id = frame;
		msg.header.stamp = stamp;

		return msg;
	}

	tf::Quaternion& getQuaternion(){return q;}
	tf::Vector3& getPosition(){return w2c.getOrigin();}

	double getDifferenceAngle(Hypothesis& other)
	{
		return this->getQuaternion().angle(other.getQuaternion());
	}

	double getDistance(Hypothesis& other)
	{
		return (this->getPosition() - other.getPosition()).length();
	}

	void setC2W(tf::Transform trans)
	{
		c2w = trans;
		w2c = c2w.inverse();
		q = w2c.getRotation();
	}

	void setW2C(tf::Transform trans)
	{
		w2c = trans;
		c2w = w2c.inverse();
		q = w2c.getRotation();
	}

	tf::Transform getW2C(){return w2c;}
	tf::Transform getC2W(){return c2w;}

	/*
	 * project point into camera frame from world
	 */
	tf::Vector3 projectPoint(tf::Vector3 in)
	{
		tf::Vector3 reproj = c2w * in;
		ROS_DEBUG_COND(reproj.z() < 0, "WARNING! POINT IS BEHIND CAMERA");
		return reproj;
	}

	/*
	 * project 3d point to pixel point from world
	 * returns -1, -1 if behind camera
	 */
	cv::Point2d projectPoint(tf::Vector3 in, cv::Mat_<float> K)
	{
		tf::Vector3 reproj = projectPoint(in);

		ROS_DEBUG_COND(reproj.z() < 0, "WARNING! POINT IS BEHIND CAMERA");

		return point2Pixel(reproj, K);
	}

	cv::Point2d projectPointNormal(tf::Vector3 in)
	{
		tf::Vector3 reproj = projectPoint(in);

		return normalizePoint(reproj);
	}

	cv::Point2d normalizePoint(tf::Vector3 pt)
	{
		return cv::Point2d((pt.x()/pt.z()), (pt.y()/pt.z()));
	}

	cv::Point2d point2Pixel(tf::Vector3 pt, cv::Mat_<float> K)
	{
		return cv::Point2d(K(0)*(pt.x()/pt.z()) + K(2), K(4)*(pt.y()/pt.z()) + K(5));
	}

	inline tf::Transform rotAndtvec2tf(cv::Mat_<double> rot, cv::Mat_<double> tvec)
	{
		tf::Transform trans;

		trans.getBasis().setValue(rot(0), rot(1), rot(2), rot(3), rot(4), rot(5), rot(6), rot(7), rot(8));
		trans.setOrigin(tf::Vector3(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));

		return trans;
	}

	inline tf::Transform rvecAndtvec2tf(cv::Mat_<double> rvec, cv::Mat_<double> tvec)
	{
		tf::Transform trans;

		cv::Mat_<double> rot;
		cv::Rodrigues(rvec, rot);

		trans.getBasis().setValue(rot(0), rot(1), rot(2), rot(3), rot(4), rot(5), rot(6), rot(7), rot(8));
		trans.setOrigin(tf::Vector3(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));

		return trans;
	}

};

std::ostream& operator<< (std::ostream& o, const Hypothesis& hyp)
{
	//return o << "Pos: x: " << hyp.getPosition().x() << " y: " << hyp.getPosition().y() << " z: " << hyp.getPosition().z() << std::endl;
	return o << "test";
}


struct MantisImage{
	cv::Mat img;
	cv::Mat K;
	std::string frame_id;
	ros::Time stamp;

	std::vector<std::vector<cv::Point> > raw_quads;

	MantisImage(){

	}

	MantisImage(cv::Mat img, cv::Mat K, std::string frame, ros::Time t)
	{
		this->K = K;
		this->img = img;
		frame_id = frame;
		stamp = t;
	}

	/*
	 * special mono computation function using only green and red channels
	 */
	cv::Mat computeMonoImage(cv::Mat in)
	{
		ROS_ASSERT(DO_RED_GREEN_TRICK);

		cv::Mat bgr[3];   //destination array
		cv::split(in,bgr);//split source

		// combine just the red and green channels into one image
		return ((bgr[1] + bgr[2]) / 2);
	}

	operator cv::Mat() const {return img;} // conversion function
};


struct Measurement{
	MantisImage img1, img2, img3;

	int detectQuadrilaterals()
	{
		int quads = detectQuadrilaterals(img1);

		return quads;
	}

	int detectQuadrilaterals(MantisImage& img)
	{
		cv::Mat mono, scaled, canny;

		cv::resize(img.img, scaled, cv::Size(img.img.cols / QUAD_DETECTION_INV_SCALE, img.img.rows / QUAD_DETECTION_INV_SCALE)); // resize the image to reduce computationaly burden.

		mono = img.computeMonoImage(scaled); // get G&R -> mono image

		cv::GaussianBlur(mono, mono, cv::Size(3, 3), 3, 3);
		cv::Canny(mono, canny, CANNY_HYSTERESIS, 3 * CANNY_HYSTERESIS, 3);
		//cv::dilate(canny, canny, cv::Mat(), cv::Point(-1, -1), 1);
		//cv::erode(canny, canny, cv::Mat(), cv::Point(-1, -1), 1);

		std::vector<std::vector<cv::Point> > contours;

		cv::findContours(canny, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

		std::vector<cv::Point> approx;
		for(int i = 0; i < contours.size(); i++)
		{
			cv::approxPolyDP(contours.at(i),approx,POLYGON_EPSILON,true);
			if(approx.size() == 4)
			{
				img.raw_quads.push_back(approx);
			}
		}

#if ULTRA_DEBUG
		cv::imshow("canny", canny);
		cv::waitKey(30);
#endif

		ROS_DEBUG_STREAM("detected " << img.raw_quads.size() << " quads");

		return img.raw_quads.size();
	}
};


#endif /* MANTIS_INCLUDE_MANTIS3_MANTIS3TYPES_H_ */
