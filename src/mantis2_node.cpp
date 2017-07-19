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

std::vector<tf::Vector3> white_test_points, green_test_points, red_test_points; // error test points
#include "mantis2/GridTestPointGeneration.h" // function to generate the testpoints

#include "mantis2/HypothesisError.h"

#include "mantis2/Mantis2HypothesisGeneration.h"

#include "mantis2/YawMarkov.h"

#include "mantis2/XYMarkovModel.h"

tf::TransformListener* tf_listener;

BaseFrameHypothesis currentBestPoseEstimate;
BaseFrameHypothesis lastPoseGuess;

XYMarkovModel xy_markov_model;

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

#define DATASET_FIX "front_camera"

void callback(const sensor_msgs::ImageConstPtr& img1, const sensor_msgs::CameraInfoConstPtr& cam1, const sensor_msgs::ImageConstPtr& img2, const sensor_msgs::CameraInfoConstPtr& cam2, const sensor_msgs::ImageConstPtr& img3, const sensor_msgs::CameraInfoConstPtr& cam3, const sensor_msgs::ImageConstPtr& img4, const sensor_msgs::CameraInfoConstPtr& cam4)
{
	ROS_DEBUG("mantis2 start");
	Measurement measurement;

	//lookup the current best guess for pose
	tf::StampedTransform w2b_st;
	try {
		tf_listener->lookupTransform(WORLD_FRAME, BASE_FRAME,
				img1->header.stamp, w2b_st);
	} catch (tf::TransformException& e) {
		ROS_ERROR_STREAM(e.what());
		ROS_INFO("will use last guess as current guess");
		w2b_st.setData(lastPoseGuess.getW2B()); // set current guess to last guess
	}

	BaseFrameHypothesis currentPoseGuess = BaseFrameHypothesis(tf::Transform(w2b_st));
	tf::Vector3 dr = (currentPoseGuess.getW2B().getOrigin() - lastPoseGuess.getW2B().getOrigin());

	//form all MantisImages
	//TODO make sure shared data is not buggy
	measurement.bottom_img = MantisImage(cv_bridge::toCvShare(img1, img1->encoding)->image, get3x3FromVector(cam1->K), img1->header.frame_id, img1->header.stamp, tf_listener);
	//measurement.img2 = MantisImage(cv_bridge::toCvShare(img2, img2->encoding)->image, get3x3FromVector(cam2->K), img2->header.frame_id, img2->header.stamp, tf_listener);
	//TODO remove after dataset is fixed
	measurement.img2 = MantisImage(cv_bridge::toCvShare(img2, img2->encoding)->image, get3x3FromVector(cam2->K), DATASET_FIX, img2->header.stamp, tf_listener);
	measurement.img3 = MantisImage(cv_bridge::toCvShare(img3, img3->encoding)->image, get3x3FromVector(cam3->K), img3->header.frame_id, img3->header.stamp, tf_listener);
	measurement.img4 = MantisImage(cv_bridge::toCvShare(img4, img4->encoding)->image, get3x3FromVector(cam4->K), img4->header.frame_id, img4->header.stamp, tf_listener);

	currentPoseGuess.measurement = &measurement; // give the currentPoseGuess the measurement

	int quad_count = measurement.detectQuadrilaterals(); // find quadrilaterals

	//now we must determine if our detected quads are good enough to be factored into the markov model sense
	bool quadrilaterals_good = true;
	BaseFrameHypothesis central_hypothesis; // if we use quadrilaterals to break down the problem this is the central hypothesis
	if(quad_count < MINIMUM_QUADRILATERALS)
	{
		quadrilaterals_good = false; // skip the rest of the quad calculations
	}
	else
	{
		//TODO further evaluate quads
		quadrilaterals_good = false;
		// generate central hypotheses with yaw closest to our current guess
		//cluster hypos by z
		//get the largest cluster and generate possible hypotheses for each
	}


	//we now have two cases for evaluation

	//case 1 evaluate the xy markov model using all possiblepositions based on the top N quadrilaterals
	if(quadrilaterals_good)
	{

	}
	//case 2 evaluate each node in the markov model and use the z height and angle from the guess
	else
	{
		ROS_DEBUG("generating all xy shifts");
		std::vector<BaseFrameHypothesis> allMarkovModelPositions = generateAllXYShiftedHypotheses(currentPoseGuess);

		ROS_DEBUG("evaluating all shifts");
		evaluateBaseFrameHypotheses(allMarkovModelPositions, false);
		ROS_DEBUG("finished evaluating");

		ROS_DEBUG("generating the markov sense model");
		cv::Mat_<double> sense_model = xy_markov_model.computeSense(allMarkovModelPositions);
		ROS_DEBUG("done computing markov sense model");

#if SUPER_DEBUG
		cv::Mat sense_render = xy_markov_model.renderModel(sense_model);
		cv::imshow("sense", sense_render);
		cv::waitKey(30);
#endif
		// convolve the model
		xy_markov_model.convolve(dr.x(), dr.y());

		// update the model
		xy_markov_model.updateModel(sense_model);

#if SUPER_DEBUG
		xy_markov_model.viewModel(xy_markov_model.P);
		std::vector<BaseFrameHypothesis> test;
		test.push_back(currentPoseGuess);
		cv::Mat gt = xy_markov_model.renderModel(xy_markov_model.computeSense(test));
		cv::imshow("ground truth", gt);
		cv::waitKey(30);
#endif
	}


#if ULTRA_DEBUG
	//test hypothesis
	//cv::imshow("test", measurement.img2.img);
	//cv::waitKey(30);
	visualizeHypothesis(currentPoseGuess, measurement.img2);
#endif

	lastPoseGuess = currentPoseGuess;

	ROS_DEBUG("mantis2 end");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mantis2");

	ros::NodeHandle nh;

	//setup the tf listener
	tf_listener = new tf::TransformListener();

	//generate the testpoints
	computeTestPoints();

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
	ss.str("");
	ss << LEFT_CAMERA_NS << "/image_rect_color";
	message_filters::Subscriber<sensor_msgs::Image> image4_sub(nh, ss.str(), 20);
	ss.str("");
	ss << LEFT_CAMERA_NS << "/camera_info";
	message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo4_sub(nh, ss.str(), 20);


	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;

	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), image1_sub, cinfo1_sub, image2_sub, cinfo2_sub, image3_sub, cinfo3_sub, image4_sub, cinfo4_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, _6, _7, _8));


	// get the transform from world to base for initialization
	ROS_INFO_STREAM("WAITING FOR TANSFORM FROM " << WORLD_FRAME << " TO " << BASE_FRAME);
	if(tf_listener->waitForTransform(WORLD_FRAME, BASE_FRAME, ros::Time(0), ros::Duration(10))){
		tf::StampedTransform w2b;
		try {
			tf_listener->lookupTransform(WORLD_FRAME, BASE_FRAME,
					ros::Time(0), w2b);
		} catch (tf::TransformException& e) {
			ROS_WARN_STREAM(e.what());
		}
		tf::Transform pose = tf::Transform(w2b);
		currentBestPoseEstimate = lastPoseGuess = BaseFrameHypothesis(pose);
		ROS_INFO_STREAM("GOT TRANSFORM WITH INITIAL POS OF: " << pose.getOrigin().x() << ", " << pose.getOrigin().y() << ", " << pose.getOrigin().z());
	}
	else
	{
		ROS_FATAL("COULD NOT GET TRANSFORM");
		ros::shutdown();
		return 1;
	}

#if SUPER_DEBUG
	xy_markov_model.viewModel(xy_markov_model.P);
#endif

	// check stuff
	ROS_ASSERT((GRID_HEIGHT - 2) * TEST_POINT_SPLIT_COUNT * GRID_WIDTH * TEST_POINT_SPLIT_COUNT >= NUMBER_RANDOM_WHITE_TEST_POINTS);
	ROS_ASSERT(GRID_WIDTH * TEST_POINT_SPLIT_COUNT >= NUMBER_RANDOM_GREEN_TEST_POINTS && GRID_WIDTH * TEST_POINT_SPLIT_COUNT >= NUMBER_RANDOM_RED_TEST_POINTS);

	//start the program
	ros::spin();

	return 0;
}
